#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from jetcobot_interfaces.action import PickerAction
from collections import deque
from bolt_fms.msg import TaskSimple
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
import asyncio, threading
import math
import transformations as t

# PickerAction의 피드백 단계를 상위 상태로 매핑
def map_phase_to_status(phase: str) -> str:
    p = (phase or "").lower()
    if p in ("approaching_source", "picking"):
        return "INPROGRESS(PICK)"
    if p in ("moving_to_target", "placing"):
        return "INPROGRESS(PLACE)"
    if p in (
            "scanning",
            "searching", 
            "moving_to_scan_position", 
            "pinky_scanning", 
            "updating_poses", 
            "clearing_pinky_data",
            "approaching_target"
            ):
        return "INPROGRESS(SCAN)"
    
    return "ASSIGNED"

class TaskStatusNode(Node):
    def __init__(self):
        super().__init__('task_status_node')

        self._client = ActionClient(self, PickerAction, 'picker_action')
        self._status_pub = self.create_publisher(String, '/task_status', 10)
        
        # 변경된 TaskSimple 메시지 구조에 맞게 수정
        self._todo_sub = self.create_subscription(TaskSimple, "/task", self.todo_callback, 10)
        
        self.detected_tags = []
        self.collecting = False
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)

        self._last_status = None
        self._busy = False
        self._seq = deque()
        self.loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self._loop_thread.start()

        self.get_logger().info("✅ TaskStatusNode started")

    # ------------------------------
    # 태그 수집 및 정렬
    # ------------------------------
    async def collect_tags(self):
            """태그를 일정 시간 동안 수집하는 비동기 함수"""
            self.get_logger().info("🔍: Collecting tags...")
            self.detected_tags.clear()
            self.collecting = True
            timeout_sec = 5.0 # 최대 대기 시간
            start_time = self.get_clock().now()
            # while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
                # if len(self.detected_tags) > 0:
                #     break
            #     await asyncio.sleep(0.1)
            await asyncio.sleep(0.5)  # 안정성 확보
            self.collecting = False
            self.get_logger().info(f"✅: Found {len(self.detected_tags)} tags, {[dt["id"] for dt in self.detected_tags]}.")

    def detections_callback(self, msg):
        """AprilTag 메시지를 받아 태그 리스트에 추가 (/detections: centre.x/y 사용)"""

        # # collecting이 True이고 아직 로그 안 찍었을 때만 출력
        # if not self._detections_logged:
        #     self.get_logger().info(f"[RX] detections={len(msg.detections)} collecting={self.collecting}")
        #     self._detections_logged = True

        pinky_bag_tags = [31, 32, 33]  # 예시: 핑키백 태그 ID 목록
        if not self.collecting:
            return
        for det in msg.detections:
            if det.id not in [t["id"] for t in self.detected_tags]:
                if det.id not in pinky_bag_tags:
                    c = det.centre
                    self.detected_tags.append({"id": det.id, "x": float(c.x), "y": float(c.y)})
                    


    def sort_tags_by_distance(self, ref_x, ref_y):
        """참조 좌표로부터의 거리순으로 태그를 정렬"""  """ 나중에 능력되면 사용 """
        return sorted(
            self.detected_tags,
            key=lambda t: math.sqrt((t["x"] - ref_x)**2 + (t["y"] - ref_y)**2),
            reverse=True  # 먼 순서로 정렬
        )


    # ------------------------------
    # 상태 보고 유틸리티
    # ------------------------------
    def publish_status(self, status: str, extra_log: str = ""):
        """새로운 상태를 /task_status 토픽으로 발행"""
        if status != self._last_status:
            msg = String()
            msg.data = status
            self._status_pub.publish(msg)
            self.get_logger().info(f"[REPORT] /task_status => {status} {extra_log}")
            self._last_status = status

    # ------------------------------
    # 메인 태스크 처리 콜백
    # ------------------------------
    def todo_callback(self, msg: TaskSimple):
        task = (msg.task_type or "").upper().strip()
        self.get_logger().info(f"Task: {task})")

        pose = msg.target_pose.pose
        ref_x = pose.position.x
        ref_y = pose.position.y

        self.pinky_num = msg.pinky_id


        # quaternion -> euler
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        roll, pitch, yaw = t.euler_from_quaternion([qx, qy, qz, qw])  # radians
        ref_yaw = yaw

        self.get_logger().info(f"📩 New Task: {task} at ({ref_x}, {ref_y}, {ref_yaw})")

        if self._busy:
            self.get_logger().warn("Busy; ignoring new task.")
            self.publish_status("ERROR", "(system busy)")
            return

        self._busy = True
        self.publish_status("PENDING")
        asyncio.run_coroutine_threadsafe(
            self.process_task(task, ref_x, ref_y, ref_yaw),
            self.loop
        )

    async def process_task(self, task_type, ref_x, ref_y, ref_yaw):
        """태스크 타입에 따른 시퀀스 생성 및 실행"""
        try:
            if task_type == "LOAD":
                
                await self.send_picker_subtask("SCAN_FRONT", -1, -1, "scan_load_pose")
                await self.send_picker_subtask("SCAN_PINKY", -1, -1, "scan_load_pose")
                await self.send_picker_subtask("SCAN_RIGHT", -1, -1, "scan_load_pose")
                await self.collect_tags()
                if not self.detected_tags:
                    self.publish_status("ERROR", "(no tags detected for LOAD)")
                    return
                
                # pick_list = self.sort_tags_by_distance(ref_x, ref_y)   # 나중에 능력되면 사용
                pick_list = self.detected_tags
                pinky_pose = ['fl', 'fr', 'rl', 'rr']
                place_list = [f"pinky{self.pinky_num}/{pose}" for pose in pinky_pose]
                # place_list = place_list + [str(pid) for pid in [t["id"] for t in pick_list]]
                sequence_steps = [
                    ("PICK_AND_PLACE", t["id"], -1, place_list[i]) 
                    for i, t in enumerate(pick_list)
                ] + [
                    ("PICK_AND_PLACE", t["id"], pick_list[i-4]["id"], "") 
                    for i, t in enumerate(pick_list[4:])
                ]
                self.start_sequence(sequence_steps)
            
            elif task_type == "UNLOAD":
                await self.send_picker_subtask("SCAN_FRONT", -1, -1, "scan_unload_pose")
                # await self.send_picker_subtask("SCAN_PINKY", -1, -1, "scan_load_pose")
                # await self.send_picker_subtask("SCAN_LEFT", -1, -1, "scan_load_pose")
                await self.collect_tags()
                if not self.detected_tags:
                    self.publish_status("ERROR", "(no tags detected for UNLOAD)")
                    return

                pick_list = self.detected_tags
                # place_list = ["ground", "ground","ground", "ground"] # + [str(pid) for pid in [t["id"] for t in pick_list]]

                place_list = ["ground_left/rr", "ground_left/rl", "ground_left/fl", "ground_left/fr"] + [str(pid) for pid in [t["id"] for t in pick_list]]
                sequence_steps = [
                    ("PICK_AND_PLACE", t["id"], -1, place_list[i])  
                    for i, t in enumerate(pick_list)
                ] + [
                    ("PICK_AND_PLACE", t["id"], pick_list[i-4]["id"], "")
                    for i, t in enumerate(pick_list[4:])
                ]
                self.start_sequence(sequence_steps)


            elif task_type == "IDLE":
                await self.send_picker_subtask("HOME", -1, -1, "")
                sequence_steps = [
                    ("HOME", -1, -1, "")  # 명령, source_tag_id, target_tag_id, target_tf_name
                ]
                self.start_sequence(sequence_steps)

            else:
                self.publish_status("ERROR", "(unknown task_type)")


        finally:
            self._busy = False

    # ------------------------------
    # 시퀀서 및 액션 래퍼
    # ------------------------------
    def start_sequence(self, steps):
        """steps: [ (command, src, tgt, tf), ... ]"""
        self._seq = deque(steps)
        self.publish_status("ASSIGNED")
        asyncio.run_coroutine_threadsafe(self.send_next_step(), self.loop)
        

    async def send_next_step(self):
        """큐의 다음 단계를 가져와서 액션 서버로 전송 (async)"""
        if not self._seq:
            self.publish_status("COMPLETE")
            self._busy = False
            return

        command, src, tgt, tf = self._seq[0]  # peek
        self.get_logger().info(f"➡️ Next step: {command}, src={src}, tgt={tgt}, tf='{tf}'")

        # 다음 서브태스크 실행 (대기)
        await self.send_picker_subtask(command, src, tgt, tf)


    async def send_picker_subtask(self, command, source_id, target_id, tf_name):
        """PickerAction 클라이언트를 사용하여 서브 태스크를 전송하고 결과를 기다림 (async)"""
        goal_msg = PickerAction.Goal()
        goal_msg.command = command
        goal_msg.source_tag_id = source_id
        goal_msg.target_tag_id = target_id
        goal_msg.target_tf_name = tf_name

        # rclpy의 wait_for_server는 동기(blocking)이므로 이벤트루프를 막지 않게 to_thread 사용
        ready = await asyncio.to_thread(self._client.wait_for_server, 10.0)  # timeout_sec=10.0
        if not ready:
            self.get_logger().error("Action server not available")
            self.publish_status("ERROR", "(server not available)")
            self._seq.clear()
            self._busy = False
            return

        # goal 전송
        self.get_logger().info("[send_picker_subtask] sending goal...")
        goal_handle = await self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        if not goal_handle.accepted:
            self.get_logger().error(f"❌ {command} goal rejected")
            self.publish_status("ERROR", "(sub-task rejected)")
            self._seq.clear()
            self._busy = False
            return

        # 결과 대기
        self.get_logger().info("[send_picker_subtask] awaiting result...")
        result = await goal_handle.get_result_async()

        if not result.result.success:
            self.get_logger().warn(f"⚠️ {command} failed: {result.result.error_message}")
            self.publish_status("ERROR", f"({result.result.error_message})")
            self._seq.clear()
            self._busy = False
            return

        self.get_logger().info(f"✅ {command} completed")

        # 다음 스텝으로
        if self._seq:
            self._seq.popleft()
            await self.send_next_step()  # 여기서도 await!
        else:
            # self.publish_status("COMPLETE")
            self._busy = False
    
    def feedback_callback(self, feedback_msg):
        """PickerAction의 피드백을 받아 상위 상태로 변환하여 발행"""
        fb = feedback_msg.feedback
        mapped = map_phase_to_status(fb.current_phase)
        self.publish_status(mapped, f"[phase={fb.current_phase}]")


def main(args=None):
    rclpy.init(args=args)
    node = TaskStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()