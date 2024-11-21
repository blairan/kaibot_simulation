import sys
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtWidgets, QtCore

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_msg = PoseStamped()

    def send_goal(self, x, y):
        """發送導航目標位置到 ROS 2 系統."""
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.frame_id = "map"  # 根據您的坐標系進行調整
        self.pose_msg.pose.position.x = x
        self.pose_msg.pose.position.y = y
        self.pose_msg.pose.position.z = 0.0
        self.pose_msg.pose.orientation.w = 1.0  # 假設朝向為正向
        self.publisher.publish(self.pose_msg)
        self.get_logger().info(f"Goal sent: x={x}, y={y}")

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("病房送藥機器人導航")
        self.setGeometry(100, 100, 400, 300)

        self.node = NavigationNode()

        # 病床按鈕
        self.bed1_button = QtWidgets.QPushButton("導航到病床 1", self)
        self.bed1_button.setGeometry(QtCore.QRect(50, 30, 300, 50))
        self.bed1_button.clicked.connect(lambda: self.navigate_to_bed(1))

        self.bed2_button = QtWidgets.QPushButton("導航到病床 2", self)
        self.bed2_button.setGeometry(QtCore.QRect(50, 90, 300, 50))
        self.bed2_button.clicked.connect(lambda: self.navigate_to_bed(2))

        self.bed3_button = QtWidgets.QPushButton("導航到病床 3", self)
        self.bed3_button.setGeometry(QtCore.QRect(50, 150, 300, 50))
        self.bed3_button.clicked.connect(lambda: self.navigate_to_bed(3))

        self.bed4_button = QtWidgets.QPushButton("導航到病床 4", self)
        self.bed4_button.setGeometry(QtCore.QRect(50, 210, 300, 50))
        self.bed4_button.clicked.connect(lambda: self.navigate_to_bed(4))

        # 已接收藥物按鈕
        self.received_button = QtWidgets.QPushButton("病人已接收藥物", self)
        self.received_button.setGeometry(QtCore.QRect(50, 270, 300, 50))
        self.received_button.clicked.connect(self.return_to_origin)

        # 記錄原點
        self.origin_x = 0.0
        self.origin_y = 0.0

    def navigate_to_bed(self, bed_number):
        """根據病床號碼導航到相應的位置."""
        if bed_number == 1:
            x, y = 1.0, 1.0  # 假設病床1的坐標
        elif bed_number == 2:
            x, y = 2.0, 1.0  # 假設病床2的坐標
        elif bed_number == 3:
            x, y = 1.0, 2.0  # 假設病床3的坐標
        elif bed_number == 4:
            x, y = 2.0, 2.0  # 假設病床4的坐標
        else:
            return
        
        self.node.send_goal(x, y)

    def return_to_origin(self):
        """返回到原點."""
        self.node.send_goal(self.origin_x, self.origin_y)

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    # 在一個獨立的執行緒中運行 ROS 2 的 spin
    executor_thread = threading.Thread(target=rclpy.spin, args=(main_window.node,), daemon=True)
    executor_thread.start()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()
