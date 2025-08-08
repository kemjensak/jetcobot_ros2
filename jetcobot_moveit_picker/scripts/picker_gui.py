#!/usr/bin/env python3
"""
Picker Action GUI Client

This GUI application provides an interface to send goals to the TagPicker action server
and monitor the real-time feedback and results.

Requirements:
- PyQt5 (sudo apt install python3-pyqt5)
- ROS2 packages
"""

import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QPushButton, QLineEdit, QTextEdit, QLabel, QComboBox,
    QGroupBox, QProgressBar, QFrame, QMessageBox, QSplitter
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QIcon

from jetcobot_interfaces.action import PickerAction


class ActionClientSignals(QObject):
    """Signals for thread-safe communication between ROS callbacks and GUI"""
    feedback_received = pyqtSignal(str, int)  # current_phase, current_tag_id
    result_received = pyqtSignal(bool, str)   # success, error_message
    goal_response_received = pyqtSignal(bool)  # goal_accepted


class PickerActionClient(Node):
    """ROS2 Action Client for Picker Action"""
    
    def __init__(self, signals: ActionClientSignals):
        super().__init__('picker_gui_client')
        self.signals = signals
        
        # Create action client
        self._action_client = ActionClient(self, PickerAction, 'picker_action')
        
        # Wait for action server
        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
        else:
            self.get_logger().info("Action server connected!")
    
    def send_goal(self, command: str, source_tag_id: int = -1, 
                  target_tag_id: int = -1, target_tf_name: str = ""):
        """Send goal to the action server"""
        if not self._action_client.server_is_ready():
            self.get_logger().error("Action server not ready!")
            return False
        
        # Create goal message
        goal_msg = PickerAction.Goal()
        goal_msg.command = command
        goal_msg.source_tag_id = source_tag_id
        goal_msg.target_tag_id = target_tag_id
        goal_msg.target_tf_name = target_tf_name
        
        self.get_logger().info(f"Sending goal: {command}, source_id={source_tag_id}, "
                              f"target_id={target_tag_id}, target_tf='{target_tf_name}'")
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        return True
    
    def _goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            self.signals.goal_response_received.emit(False)
            return
        
        self.get_logger().info("Goal accepted by server")
        self.signals.goal_response_received.emit(True)
        
        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Feedback: phase={feedback.current_phase}, tag_id={feedback.current_tag_id}")
        self.signals.feedback_received.emit(feedback.current_phase, feedback.current_tag_id)
    
    def _get_result_callback(self, future):
        """Handle final result from action server"""
        result = future.result().result
        self.get_logger().info(f"Result: success={result.success}, error='{result.error_message}'")
        self.signals.result_received.emit(result.success, result.error_message)


class ROSThread(QThread):
    """Thread for running ROS2 executor"""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(node)
        self._running = True
    
    def run(self):
        """Run the ROS2 executor"""
        while self._running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
    
    def stop(self):
        """Stop the executor"""
        self._running = False
        self.executor.shutdown()


class PickerGUI(QMainWindow):
    """Main GUI Window for Picker Action Client"""
    
    # Available commands
    COMMANDS = [
        "HOME",
        "SCAN", 
        "SCAN_FRONT",
        "SCAN_LEFT", 
        "SCAN_RIGHT",
        "PICK_AND_PLACE"
    ]
    
    # Phase descriptions
    PHASE_DESCRIPTIONS = {
        "searching": "🔍 Searching for tags...",
        "approaching_source": "➡️ Approaching source tag...",
        "picking": "🤏 Picking object...",
        "moving_to_target": "🚚 Moving to target location...",
        "placing": "📦 Placing object...",
        "completed": "✅ Operation completed!",
        "moving_to_home": "🏠 Moving to home position...",
        "moving_to_scan_position": "📍 Moving to scan position..."
    }
    
    def __init__(self):
        super().__init__()
        self.action_client = None
        self.ros_thread = None
        self.current_goal_handle = None
        
        self.init_ros()
        self.init_ui()
        self.connect_signals()
        
        # Status timer for periodic updates
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(100)  # Update every 100ms
    
    def init_ros(self):
        """Initialize ROS2 components"""
        # Initialize ROS2
        rclpy.init()
        
        # Create signals for thread-safe communication
        self.signals = ActionClientSignals()
        
        # Create action client node
        self.action_client = PickerActionClient(self.signals)
        
        # Start ROS thread
        self.ros_thread = ROSThread(self.action_client)
        self.ros_thread.start()
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Picker Action GUI")
        self.setGeometry(100, 100, 900, 700)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create splitter for resizable panes
        splitter = QSplitter()
        central_widget_layout = QVBoxLayout(central_widget)
        central_widget_layout.addWidget(splitter)
        
        # Left panel - Goal configuration
        self.init_goal_panel(splitter)
        
        # Right panel - Status and feedback
        self.init_status_panel(splitter)
        
        # Set splitter proportions
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
    
    def init_goal_panel(self, parent):
        """Initialize goal configuration panel"""
        goal_widget = QWidget()
        goal_layout = QVBoxLayout(goal_widget)
        
        # Goal Configuration Group
        goal_group = QGroupBox("Goal Configuration")
        goal_group_layout = QGridLayout(goal_group)
        
        # Command selection
        goal_group_layout.addWidget(QLabel("Command:"), 0, 0)
        self.command_combo = QComboBox()
        self.command_combo.addItems(self.COMMANDS)
        self.command_combo.currentTextChanged.connect(self.on_command_changed)
        goal_group_layout.addWidget(self.command_combo, 0, 1)
        
        # Source Tag ID
        goal_group_layout.addWidget(QLabel("Source Tag ID:"), 1, 0)
        self.source_tag_edit = QLineEdit()
        self.source_tag_edit.setPlaceholderText("Enter source tag ID (for PICK_AND_PLACE)")
        goal_group_layout.addWidget(self.source_tag_edit, 1, 1)
        
        # Target Tag ID
        goal_group_layout.addWidget(QLabel("Target Tag ID:"), 2, 0)
        self.target_tag_edit = QLineEdit()
        self.target_tag_edit.setPlaceholderText("Enter target tag ID (optional)")
        goal_group_layout.addWidget(self.target_tag_edit, 2, 1)
        
        # Target TF Frame
        goal_group_layout.addWidget(QLabel("Target TF Frame:"), 3, 0)
        self.target_tf_edit = QLineEdit()
        self.target_tf_edit.setPlaceholderText("Enter TF frame name (alternative to tag ID)")
        goal_group_layout.addWidget(self.target_tf_edit, 3, 1)
        
        goal_layout.addWidget(goal_group)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.send_goal_btn = QPushButton("🚀 Send Goal")
        self.send_goal_btn.clicked.connect(self.send_goal)
        self.send_goal_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        
        self.clear_btn = QPushButton("🗑️ Clear")
        self.clear_btn.clicked.connect(self.clear_inputs)
        
        button_layout.addWidget(self.send_goal_btn)
        button_layout.addWidget(self.clear_btn)
        goal_layout.addLayout(button_layout)
        
        # Add stretch to push everything to top
        goal_layout.addStretch()
        
        parent.addWidget(goal_widget)
    
    def init_status_panel(self, parent):
        """Initialize status and feedback panel"""
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        # Current Status Group
        status_group = QGroupBox("Current Status")
        status_group_layout = QVBoxLayout(status_group)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        status_group_layout.addWidget(self.progress_bar)
        
        # Current phase
        self.phase_label = QLabel("Ready to send goals")
        self.phase_label.setFont(QFont("Arial", 12, QFont.Bold))
        status_group_layout.addWidget(self.phase_label)
        
        # Current tag ID
        self.tag_id_label = QLabel("")
        status_group_layout.addWidget(self.tag_id_label)
        
        status_layout.addWidget(status_group)
        
        # Feedback/Result Group
        feedback_group = QGroupBox("Feedback & Results")
        feedback_layout = QVBoxLayout(feedback_group)
        
        self.feedback_text = QTextEdit()
        self.feedback_text.setReadOnly(True)
        self.feedback_text.setMaximumHeight(200)
        feedback_layout.addWidget(self.feedback_text)
        
        status_layout.addWidget(feedback_group)
        
        # Logs Group
        logs_group = QGroupBox("Logs")
        logs_layout = QVBoxLayout(logs_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        logs_layout.addWidget(self.log_text)
        
        status_layout.addWidget(logs_group)
        
        parent.addWidget(status_widget)
    
    def connect_signals(self):
        """Connect ROS signals to GUI slots"""
        self.signals.feedback_received.connect(self.on_feedback_received)
        self.signals.result_received.connect(self.on_result_received)
        self.signals.goal_response_received.connect(self.on_goal_response_received)
    
    def on_command_changed(self, command):
        """Handle command selection change"""
        # Enable/disable fields based on command
        is_pick_and_place = (command == "PICK_AND_PLACE")
        
        self.source_tag_edit.setEnabled(is_pick_and_place)
        self.target_tag_edit.setEnabled(is_pick_and_place)
        self.target_tf_edit.setEnabled(is_pick_and_place)
        
        if not is_pick_and_place:
            self.source_tag_edit.clear()
            self.target_tag_edit.clear()
            self.target_tf_edit.clear()
    
    def send_goal(self):
        """Send goal to action server"""
        command = self.command_combo.currentText()
        
        # Parse inputs
        source_tag_id = -1
        target_tag_id = -1
        target_tf_name = ""
        
        try:
            if self.source_tag_edit.text().strip():
                source_tag_id = int(self.source_tag_edit.text().strip())
            
            if self.target_tag_edit.text().strip():
                target_tag_id = int(self.target_tag_edit.text().strip())
            
            target_tf_name = self.target_tf_edit.text().strip()
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Tag IDs must be integers!")
            return
        
        # Validate PICK_AND_PLACE requirements
        if command == "PICK_AND_PLACE":
            if source_tag_id == -1:
                QMessageBox.warning(self, "Missing Input", "Source Tag ID is required for PICK_AND_PLACE!")
                return
            
            if target_tag_id == -1 and not target_tf_name:
                QMessageBox.warning(self, "Missing Input", 
                                  "Either Target Tag ID or Target TF Frame is required for PICK_AND_PLACE!")
                return
        
        # Send goal
        if self.action_client.send_goal(command, source_tag_id, target_tag_id, target_tf_name):
            self.log_message(f"Sending goal: {command}")
            self.send_goal_btn.setEnabled(False)
            self.progress_bar.setVisible(True)
            self.progress_bar.setRange(0, 0)  # Indeterminate progress
        else:
            QMessageBox.critical(self, "Error", "Failed to send goal. Check if action server is running!")
    
    def clear_inputs(self):
        """Clear all input fields"""
        self.source_tag_edit.clear()
        self.target_tag_edit.clear()
        self.target_tf_edit.clear()
        self.command_combo.setCurrentIndex(0)
    
    def on_feedback_received(self, current_phase: str, current_tag_id: int):
        """Handle feedback from action server"""
        phase_desc = self.PHASE_DESCRIPTIONS.get(current_phase, f"Phase: {current_phase}")
        self.phase_label.setText(phase_desc)
        
        if current_tag_id >= 0:
            self.tag_id_label.setText(f"Processing Tag ID: {current_tag_id}")
        else:
            self.tag_id_label.setText("")
        
        # Add to feedback text
        feedback_msg = f"[FEEDBACK] {phase_desc}"
        if current_tag_id >= 0:
            feedback_msg += f" (Tag ID: {current_tag_id})"
        
        self.feedback_text.append(feedback_msg)
        self.log_message(f"Feedback: {current_phase}")
    
    def on_result_received(self, success: bool, error_message: str):
        """Handle result from action server"""
        self.progress_bar.setVisible(False)
        self.send_goal_btn.setEnabled(True)
        
        if success:
            self.phase_label.setText("✅ Goal completed successfully!")
            result_msg = "[RESULT] ✅ SUCCESS"
            self.log_message("Goal completed successfully!")
        else:
            self.phase_label.setText("❌ Goal failed!")
            result_msg = f"[RESULT] ❌ FAILED: {error_message}"
            self.log_message(f"Goal failed: {error_message}")
        
        self.feedback_text.append(result_msg)
        self.tag_id_label.setText("")
    
    def on_goal_response_received(self, accepted: bool):
        """Handle goal response from action server"""
        if accepted:
            self.log_message("Goal accepted by server")
        else:
            self.log_message("Goal rejected by server")
            self.progress_bar.setVisible(False)
            self.send_goal_btn.setEnabled(True)
            QMessageBox.warning(self, "Goal Rejected", "The goal was rejected by the action server!")
    
    def log_message(self, message: str):
        """Add message to log"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
    
    def update_status(self):
        """Periodic status updates"""
        # Could add connection status, server availability, etc.
        pass
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.ros_thread:
            self.ros_thread.stop()
            self.ros_thread.wait()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        event.accept()


def main():
    """Main function"""
    app = QApplication(sys.argv)
    
    # Set application properties
    app.setApplicationName("Picker Action GUI")
    app.setApplicationVersion("1.0")
    
    try:
        # Create and show main window
        window = PickerGUI()
        window.show()
        
        # Run application
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Error starting application: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
