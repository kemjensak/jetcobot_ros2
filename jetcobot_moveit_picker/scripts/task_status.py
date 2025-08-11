#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from jetcobot_interfaces.action import PickerAction
from jetcobot_interfaces.msg import ArmTaskStatus, TaskSimple
from collections import deque

def map_phase_to_status(phase: str) -> str:
    p = (phase or "").lower()
    if p in ("approaching_source", "picking"):
        return "INPROGRESS(PICK)"
    if p in ("moving_to_target", "placing"):
        return "INPROGRESS(PLACE)"
    return "ASSIGNED"

class TaskStatusNode(Node):
    def __init__(self):
        super().__init__('task_status_node')

        # 기본 태그 파라미터(나중에 다른 함수/알고리즘으로 대체 가능)
        self.declare_parameter('load_source_tag_id', 8)
        self.declare_parameter('load_target_tag_id', 2)
        self.declare_parameter('unload_source_tag_id', 2)
        self.declare_parameter('unload_target_tag_id', 8)

        self.load_source_tag_id = int(self.get_parameter('load_source_tag_id').value)
        self.load_target_tag_id = int(self.get_parameter('load_target_tag_id').value)
        self.unload_source_tag_id = int(self.get_parameter('unload_source_tag_id').value)
        self.unload_target_tag_id = int(self.get_parameter('unload_target_tag_id').value)

        self._client = ActionClient(self, PickerAction, 'picker_action')
        self._status_pub = self.create_publisher(ArmTaskStatus, '/arm_task', 10)
        self._todo_sub = self.create_subscription(TaskSimple, "/task", self.todo_callback, 10)

        self._last_status = None
        self._busy = False
        self._seq = deque()

        self.get_logger().info("✅ TaskStatusNode up (action: picker_action, type: PickerAction)")

    # ---------- 확장 포인트(훅) ----------
    def select_tags_for_load(self):
        """LOAD 작업에 사용할 (source_tag_id, target_tag_id, target_tf_name) 반환.
        나중에 비전/DB/스케줄러로 교체 가능."""
        return self.load_source_tag_id, self.load_target_tag_id, ""

    def select_tags_for_unload(self):
        """UNLOAD 작업에 사용할 (source_tag_id, target_tag_id, target_tf_name) 반환."""
        return self.unload_source_tag_id, self.unload_target_tag_id, ""

    # 필요하면 여기서 사전 SCAN, 재스캔, 경로 검증 등도 훅으로 추가 가능
    # def pre_action_hook(self, task_type: str): pass
    # def post_action_hook(self, task_type: str, success: bool): pass

    # ---------- 공통 유틸 ----------
    def publish_status(self, status: str, extra_log: str = ""):
        if status != self._last_status:
            msg = ArmTaskStatus()
            msg.status = status
            self._status_pub.publish(msg)
            self.get_logger().info(f"[REPORT] /arm_task => {status} {extra_log}")
            self._last_status = status

    # ---------- 메인 플로우 ----------
    def todo_callback(self, msg: TaskSimple):  #ArmTodoTask
        task = (msg.task_type or "").upper().strip()
        self.get_logger().info(f"📩 /arm_todo_task: task_type={task}")

        if self._busy:
            self.get_logger().warn("Busy; ignoring new task.")
            return

        self.publish_status("PENDING")

        if task == "IDLE":
            # IDLE: HOME만 실행
            self._busy = True
            self.start_sequence([("HOME", -1, -1, "")])
            return

        if task == "LOAD":
            # 원하는 순서: HOME → SCAN → PnP(태그→태그)
            src, tgt, tf = self.select_tags_for_load()
            self._busy = True
            self.start_sequence([
                ("HOME", -1, -1, ""),
                ("SCAN", -1, -1, ""),
                ("PICK_AND_PLACE", src, tgt, tf),
            ])
            return

        if task == "UNLOAD":
            src, tgt, tf = self.select_tags_for_unload()
            self._busy = True
            self.start_sequence([
                ("HOME", -1, -1, ""),
                ("SCAN", -1, -1, ""),
                ("PICK_AND_PLACE", src, tgt, tf),
            ])
            return

        self.publish_status("ERROR", "(unknown task_type)")


    # ---- 시퀀서 ----
    def start_sequence(self, steps):
        """steps: [ (command, src, tgt, tf), ... ]"""
        self._seq = deque(steps)
        self.publish_status("ASSIGNED")
        self.send_next_step()

    def send_next_step(self):
        if not self._seq:
            # 모든 단계 완료
            self.publish_status("COMPLETE")
            self._busy = False
            return

        command, src, tgt, tf = self._seq[0]  # peek
        self.get_logger().info(f"➡️ Next step: {command}, src={src}, tgt={tgt}, tf='{tf}'")

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            self.publish_status("ERROR", "(server not available)")
            self._seq.clear(); self._busy = False
            return

        goal = PickerAction.Goal()
        goal.command = command
        goal.source_tag_id = src
        goal.target_tag_id = tgt
        goal.target_tf_name = tf

        # 홈/스캔 단계도 피드백은 올 수 있으니 상태는 콜백에서 업데이트됨
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    # ---------- 액션 래퍼 ----------
    def send_goal(self, command, source_tag_id, target_tag_id, target_tf_name):
        self.get_logger().info("Waiting for action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            self.publish_status("ERROR", "(server not available)")
            self._busy = False
            return

        self.publish_status("ASSIGNED")

        goal_msg = PickerAction.Goal()
        goal_msg.command = command
        goal_msg.source_tag_id = source_tag_id
        goal_msg.target_tag_id = target_tag_id
        goal_msg.target_tf_name = target_tf_name

        self.get_logger().info(
            f"📤 Sending goal: cmd={command}, src={source_tag_id}, tgt={target_tag_id}, tf='{target_tf_name}'"
        )
        future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            self.publish_status("ERROR", "(rejected)")
            self._busy = False
            return
        self.get_logger().info("✅ Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        mapped = map_phase_to_status(fb.current_phase)
        self.publish_status(mapped, f"[phase={fb.current_phase}, tag={fb.current_tag_id}]")


    def result_callback(self, future):
        res = future.result().result
        if not res.success:
            self.publish_status("ERROR", f"({res.error_message})")
            self._seq.clear(); self._busy = False
            return

        # 방금 단계 성공 → 큐에서 꺼내고 다음 단계 전송
        finished = self._seq.popleft()
        self.get_logger().info(f"✅ Step done: {finished[0]}")
        self.send_next_step()

        
def main():
    rclpy.init()
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
