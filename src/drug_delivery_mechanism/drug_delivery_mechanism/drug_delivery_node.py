import sys
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer, QTime

class BedNavigation(Node):
    def __init__(self):
        super().__init__('bed_navigation')
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
        self.setWindowTitle("病房送藥機器人")
        self.setGeometry(100, 100, 400, 550)

        self.node = BedNavigation()

        # 創建病床按鈕框架
        self.bed_frame = QtWidgets.QFrame(self)  # 創建一個QFrame來包含按鈕
        self.bed_frame.setGeometry(QtCore.QRect(20, 20, 360, 300))  # 設定框架的位置和大小
        self.bed_frame.setStyleSheet("border: 2px solid red;")  # 設定邊框樣式，這裡為紅色邊框

        # 返回配藥間按鈕
        self.return_button = QtWidgets.QPushButton("返回配藥間", self.bed_frame)  # 創建返回按鈕
        self.return_button.setGeometry(QtCore.QRect(0, 0, 120, 50))  # 設定按鈕位置和大小
        self.return_button.setStyleSheet("background-color: lightyellow; color: black; font-size: 16px; font-weight: bold;")  # 設定按鈕樣式
        self.return_button.clicked.connect(self.return_to_pharmacy)  # 連接按鈕點擊事件

        # 病床按鈕
        self.bed_buttons = []
        for i in range(1, 5):
            button = QtWidgets.QPushButton(f"導航到病床 {i}", self.bed_frame)  # 將按鈕添加到bed_frame中
            button.setGeometry(QtCore.QRect(30, 60 + (i-1) * 60, 300, 50))  # 調整按鈕位置和大小
            button.setStyleSheet("background-color: lightblue; color: black; font-size: 18px; font-weight: bold;")  # 設定按鈕顏色和字型
            button.clicked.connect(lambda _, num=i: self.navigate_to_bed(num))  # 連接按鈕點擊事件
            self.bed_buttons.append(button)  # 將按鈕添加到列表中

        # 創建病人接收藥物按鈕框架
        self.patient_frame = QtWidgets.QFrame(self)
        self.patient_frame.setGeometry(QtCore.QRect(20, 330, 360, 120))
        self.patient_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)  # 設定病人框架樣式

        # 提示標籤
        self.text_label = QtWidgets.QLabel("已接收請按「病人已接收藥物」按鈕", self.patient_frame)
        self.text_label.setGeometry(QtCore.QRect(30, 15, 300, 30))  # 設定標籤位置和大小
        self.text_label.setStyleSheet("font-size: 18px; font-weight: bold; color: black;")  # 設定字型顏色和大小

        # 已接收藥物按鈕
        self.received_button = QtWidgets.QPushButton("病人已接收藥物", self.patient_frame)
        self.received_button.setGeometry(QtCore.QRect(30, 55, 300, 50))
        self.received_button.setStyleSheet("background-color: lightgreen; color: black; font-size: 18px; font-weight: bold;")  # 設定按鈕顏色和字型
        self.received_button.clicked.connect(self.patient_received_medication)  # 連接按鈕點擊事件

        # 記錄原點
        self.origin_x = 0.0
        self.origin_y = 0.0

        # 當前時間顯示
        self.time_label = QtWidgets.QLabel("當前時間: ", self)
        self.time_label.setGeometry(QtCore.QRect(50, 455, 300, 50))
        self.update_time()  # 初始化顯示當前時間

        # 設定目標時間
        self.target_time_edit = QtWidgets.QTimeEdit(self)
        self.target_time_edit.setGeometry(QtCore.QRect(50, 500, 100, 30))
        self.target_time_edit.setTime(QTime.currentTime())  # 設定當前時間為預設值

        # 開始按鈕
        self.start_button = QtWidgets.QPushButton("開始自動發送藥物", self)
        self.start_button.setGeometry(QtCore.QRect(160, 500, 190, 30))
        self.start_button.clicked.connect(self.start_auto_send)

        # 設置定時器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_time)
        self.timer.start(1000)  # 每秒檢查一次

        # 狀態變數
        self.current_bed = 0  # 當前病床編號
        self.auto_send_active = False  # 自動發送藥物狀態
    
    def return_to_pharmacy(self):
        print("返回配藥間按鈕被按下。")
        # 在這裡添加返回配藥間的邏輯
        self.return_to_origin()

    def navigate_to_bed(self, number):
        if number == 1:
            x, y = 4.0, 2.0
        elif number == 2:
            x, y = -3.0, 2.0  # 假設病床2的坐標
        elif number == 3:
            x, y = -3.5, -2.0  # 假設病床3的坐標
        elif number == 4:
            x, y = -4.0, -4.5  # 假設病床4的坐標
        
        self.node.send_goal(x, y)  # 發送導航請求
    
    def return_to_origin(self):
        self.node.send_goal(self.origin_x, self.origin_y)

    def update_time(self):
        """更新當前時間顯示."""
        current_time = QTime.currentTime().toString()
        self.time_label.setText(f"當前時間: {current_time}")

    def check_time(self):
        """檢查當前時間是否達到預定的發送藥物時間."""
        self.update_time()  # 更新當前時間顯示
        current_time = QTime.currentTime()  # 獲取當前時間
        target_time = self.target_time_edit.time()  # 獲取用戶設定的目標時間
        
        if self.auto_send_active and current_time.hour() == target_time.hour() and current_time.minute() == target_time.minute():
            self.current_bed = 1  # 從1號病床開始
            self.navigate_to_bed(self.current_bed)  # 自動導航至第一號病床

    def patient_received_medication(self):
        """病人已接收藥物的邏輯."""
        if self.current_bed < 4:  # 如果當前病床小於4，導航至下一個病床
            self.current_bed += 1
            self.navigate_to_bed(self.current_bed)  # 導航至下一個病床
        else:
            self.node.get_logger().info("所有病床的藥物已發送完畢。")
            self.auto_send_active = False  # 停止自動發送藥物
            self.return_to_origin()

    def start_auto_send(self):
        """開始自動發送藥物的功能."""
        self.auto_send_active = True
        self.node.get_logger().info("自動發送藥物功能已啟動。")

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    # 在一個獨立的執行緒中運行 ROS 2 的 spin
    executor_thread = threading.Thread(target=rclpy.spin, args=(main_window.node,), daemon=True)
    executor_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
