#pragma once

namespace jetcobot_picker {

struct TimingConstants {
    static constexpr int OPERATION_DELAY_MS = 500;
    static constexpr int GRIPPER_CLOSE_DELAY_MS = 1000;
    static constexpr int STABILIZE_DELAY_MS = 1000;
    static constexpr double TAG_COLLECTION_TIME = 3.0;
};

struct GripperPositions {
    static constexpr int FULLY_OPEN = 255;
    static constexpr int FULLY_CLOSED = 0;
    static constexpr int HOLDING_POSITION = 160;
    static constexpr int PICKING_POSITION = 90;
};

struct MovementConstants {
    static constexpr double EEF_STEP = 0.01;
    static constexpr double MIN_PATH_FRACTION = 0.8;
    static constexpr double MIN_DISTANCE_TO_BASE = 1e-6;
    static constexpr double CAM_HEIGHT = 0.05;
    static constexpr double APPROACH_HEIGHT = 0.05;
    static constexpr double PICK_HEIGHT = -0.01;
    static constexpr double PLACE_HEIGHT = 0.01;
    static constexpr double LIFT_HEIGHT = 0.05;
};

} // namespace jetcobot_picker
