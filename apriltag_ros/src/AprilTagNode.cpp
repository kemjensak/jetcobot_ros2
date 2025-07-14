// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>

// threading
#include <thread>
#include <atomic>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    // Camera and processing
    cv::VideoCapture camera;
    std::thread camera_thread;
    std::atomic<bool> running;
    std::atomic<bool> camera_resolution_set;
    std::string camera_device_str;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
    
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    const rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void cameraLoop();
    void processFrame(const cv::Mat& frame);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    running(false),
    camera_resolution_set(false),
    // topics
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    sub_camera_info(create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 
        rclcpp::QoS(1), 
        std::bind(&AprilTagNode::onCameraInfo, this, std::placeholders::_1))),
    tf_broadcaster(this)
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));
    
    // Camera device parameter - can be device path (e.g., "/dev/jetcocam0") or index (e.g., "0")
    camera_device_str = declare_parameter("camera_device", "/dev/jetcocam0", descr("camera device path or index", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }

    // Camera will be initialized when camera_info is received
    RCLCPP_INFO(get_logger(), "Waiting for camera_info to initialize camera...");
    
    // Start camera thread (it will wait for camera to be opened)
    running = true;
    camera_thread = std::thread(&AprilTagNode::cameraLoop, this);
    
    RCLCPP_INFO(get_logger(), "AprilTag detector created, waiting for camera_info");
}

AprilTagNode::~AprilTagNode()
{
    // Stop camera thread
    running = false;
    if (camera_thread.joinable()) {
        camera_thread.join();
    }
    
    // Release camera
    if (camera.isOpened()) {
        camera.release();
    }
    
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::cameraLoop()
{
    cv::Mat frame;
    
    while (running && rclcpp::ok()) {
        if (!camera.isOpened()) {
            // Camera not opened yet, wait
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (!camera.read(frame)) {
            RCLCPP_WARN(get_logger(), "Failed to read frame from camera");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        if (!frame.empty()) {
            processFrame(frame);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AprilTagNode::processFrame(const cv::Mat& frame)
{
    if (!camera_info) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No camera info received yet");
        return;
    }
    
    // Wait for camera resolution to be set from camera_info
    if (!camera_resolution_set) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera resolution to be set from camera_info");
        return;
    }

    // camera intrinsics for rectified images
    const std::array<double, 4> intrinsics = {
        camera_info->p[0], camera_info->p[5], 
        camera_info->p[2], camera_info->p[6]
    };

    // check for valid intrinsics
    const bool calibrated = camera_info->width && camera_info->height &&
                            intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // convert to 8bit monochrome image
    cv::Mat img_uint8;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, img_uint8, cv::COLOR_BGR2GRAY);
    } else {
        img_uint8 = frame;
    }

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header.stamp = this->get_clock()->now();
    msg_detections.header.frame_id = camera_info->header.frame_id;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_detections.header;
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            tf.transform = estimate_pose(det, intrinsics, size);
            tfs.push_back(tf);
        }
    }

    pub_detections->publish(msg_detections);

    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}

void AprilTagNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_info = msg;
    
    // Initialize camera only once when camera_info is first received
    if (!camera_resolution_set && !camera.isOpened() && msg->width > 0 && msg->height > 0) {
        // Try to open camera
        bool camera_opened = false;
        try {
            // Try to parse as integer first (for device index)
            int device_index = std::stoi(camera_device_str);
            camera.open(device_index);
            if (camera.isOpened()) {
                camera_opened = true;
                RCLCPP_INFO(get_logger(), "Opened camera device by index: %d", device_index);
            }
        } catch (const std::exception&) {
            // Not a valid integer, treat as device path
        }
        
        if (!camera_opened) {
            // Try to open as device path
            camera.open(camera_device_str);
            if (camera.isOpened()) {
                camera_opened = true;
                RCLCPP_INFO(get_logger(), "Opened camera device by path: %s", camera_device_str.c_str());
            }
        }
        
        if (!camera_opened) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera device: %s", camera_device_str.c_str());
            return;
        }
        
        // Set camera resolution from camera_info
        camera.set(cv::CAP_PROP_FRAME_WIDTH, msg->width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, msg->height);
        
        // Verify resolution was set
        int actual_width = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_height = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_HEIGHT));
        RCLCPP_INFO(get_logger(), "Camera resolution set to: %dx%d (requested from camera_info: %dx%d)", 
                    actual_width, actual_height, msg->width, msg->height);
        
        camera_resolution_set = true;
        RCLCPP_INFO(get_logger(), "AprilTag detector started with camera device: %s", camera_device_str.c_str());
    }
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}
