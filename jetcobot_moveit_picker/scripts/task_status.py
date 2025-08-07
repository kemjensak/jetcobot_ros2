#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from jetcobot_interfaces.action import TaskAction, PickerAction

from apriltag_msgs.msg import AprilTagDetectionArray
import asyncio
import math

class TaskStatusNode(Node):
    def __init__(self):
        super().__init__('task_status_node')

        # Action Server (관제 → StatusNode)
        self._server = ActionServer(
            self, TaskAction, '/task_action', execute_callback=self.execute_task_callback
        )

        # Action Client (StatusNode → PickAndPlaceNode)
        self._client = ActionClient(self, PickerAction, '/pick_and_place_action')

        # Detection 구독
        self.detected_tags = []
        self.collecting = False
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)

        self.get_logger().info("✅ TaskStatusNode started")

    # ------------------------------
    # Action Goal (관제 요청 처리)
    # ------------------------------
    async def execute_task_callback(self, goal_handle):
        task_type = goal_handle.request.task_type.upper()
        x, y, yaw = goal_handle.request.x, goal_handle.request.y, goal_handle.request.yaw
        self.get_logger().info(f"📩 New Task: {task_type} ({x},{y},{yaw})")

        if task_type == "LOAD":
            result = await self.handle_load_task(goal_handle, x, y, yaw)
        elif task_type == "UNLOAD":
            result = await self.handle_unload_task(goal_handle, x, y, yaw)
        else:
            goal_handle.abort()
            result = TaskAction.Result()
            result.success = False
            result.message = f"Unknown task type: {task_type}"
        return result

    # ------------------------------
    # LOAD 프로세스
    # ------------------------------
    async def handle_load_task(self, goal_handle, x, y, yaw):
        # 1) SCAN_LOAD
        await self.send_pickplace_subtask("SCAN", -1, -1, "scan_load_pose", goal_handle)
        await self.collect_tags()
        if not self.detected_tags:
            return self.make_result(goal_handle, False, "No tags detected during LOAD")

        # 2) Pick list
        pick_list = self.sort_tags_by_distance(x, y)
        # 3) Place list
        place_list = ["bag1", "bag2"] + [str(pid) for pid in [t["id"] for t in pick_list]]

        # 4) 반복 Pick&Place
        for src_id, target_tf in zip([t["id"] for t in pick_list], place_list):
            await self.send_pickplace_subtask("PICK_AND_PLACE", src_id, -1, target_tf, goal_handle)

        return self.make_result(goal_handle, True, "LOAD task completed successfully")

    # ------------------------------
    # UNLOAD 프로세스
    # ------------------------------
    async def handle_unload_task(self, goal_handle, x, y, yaw):
        # 1) SCAN_UNLOAD
        await self.send_pickplace_subtask("SCAN", -1, -1, "scan_unload_pose", goal_handle)
        await self.collect_tags()
        if not self.detected_tags:
            return self.make_result(goal_handle, False, "No tags detected during UNLOAD")

        # 2) Pick list
        pick_list = self.sort_tags_by_distance(x, y)
        # 3) Place list
        place_list = ["ground1", "ground2"] + [str(pid) for pid in [t["id"] for t in pick_list]]

        # 4) 반복 Pick&Place
        for src_id, target_tf in zip([t["id"] for t in pick_list], place_list):
            await self.send_pickplace_subtask("PICK_AND_PLACE", src_id, -1, target_tf, goal_handle)

        return self.make_result(goal_handle, True, "UNLOAD task completed successfully")

    # ------------------------------
    # 태그 수집
    # ------------------------------
    async def collect_tags(self):
        self.collecting = True
        await asyncio.sleep(0.8)
        self.collecting = False

    def detections_callback(self, msg):
        if not self.collecting:
            return
        self.detected_tags.clear()
        for det in msg.detections:
            tag_id = det.id
            pose = det.pose.pose.pose
            self.detected_tags.append({"id": tag_id, "x": pose.position.x, "y": pose.position.y})

    # ------------------------------
    # 거리순 정렬
    # ------------------------------
    def sort_tags_by_distance(self, ref_x, ref_y):
        return sorted(
            self.detected_tags,
            key=lambda t: math.sqrt((t["x"] - ref_x)**2 + (t["y"] - ref_y)**2)
        )

    # ------------------------------
    # Pick&Place 서브액션 호출
    # ------------------------------
    async def send_pickplace_subtask(self, command, source_id, target_id, tf_name, goal_handle=None):
        goal_msg = PickerAction.Goal()
        goal_msg.command = command
        goal_msg.source_tag_id = source_id
        goal_msg.target_tag_id = target_id
        goal_msg.target_tf_name = tf_name
        await self._client.wait_for_server()
        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=lambda fb: self.relay_feedback(goal_handle, fb))
        goal_handle_pp = await send_goal_future
        result_future = goal_handle_pp.get_result_async()
        result = await result_future
        if not result.result.success:
            self.get_logger().warn(f"⚠️ {command} failed: {result.result.error_message}")
        else:
            self.get_logger().info(f"✅ {command} completed")

    def relay_feedback(self, goal_handle, fb):
        if goal_handle is None:
            return
        feedback = TaskAction.Feedback()
        feedback.status = fb.feedback.current_phase
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"🔄 Feedback relay: {feedback.status}")

    def make_result(self, goal_handle, success, msg):
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = TaskAction.Result()
        result.success = success
        result.message = msg
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TaskStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
