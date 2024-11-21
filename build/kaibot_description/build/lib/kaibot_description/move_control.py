import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class MoveSpeedNode(Node):
    def __init__(self):
        super().__init__("move_speed")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Node initialized, ready to move")
        self.liner_speed = 0.0  # 初始線速度
        self.angular_speed = 0.0  # 初始角速度

    def forward(self):
        msg = Twist()
        self.liner_speed = 0.5  # 設定前進速度
        self.angular_speed = 0.0  # 設定角速度
        msg.linear.x = self.liner_speed
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)
        self.get_logger().info("Move forward")
        return self.liner_speed, self.angular_speed  # 返回當前速度

    def right(self):
        msg = Twist()
        self.liner_speed = 0.0  # 停止前進速度
        self.angular_speed = -0.7  # 設定右轉速度
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)
        self.get_logger().info("Turn right")
        return self.liner_speed, self.angular_speed  # 返回當前速度

    def left(self):
        msg = Twist()
        self.liner_speed = 0.0  # 停止前進速度
        self.angular_speed = 0.7  # 設定左轉速度
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)
        self.get_logger().info("Turn left")
        return self.liner_speed, self.angular_speed  # 返回當前速度

    def backward(self):
        msg = Twist()
        self.liner_speed = -0.5  # 設定後退速度
        self.angular_speed = 0.0  # 停止轉動
        msg.linear.x = self.liner_speed
        self.pub.publish(msg)
        self.get_logger().info("Move backward")
        return self.liner_speed, self.angular_speed  # 返回當前速度

    def stop(self):
        msg = Twist()
        self.liner_speed = 0.0  # 停止線速度
        self.angular_speed = 0.0  # 停止角速度
        msg.linear.x = self.liner_speed
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)
        self.get_logger().info("Stop")
        return self.liner_speed, self.angular_speed  # 返回當前速度

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1018, 240)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(440, 10, 101, 61))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(550, 80, 101, 61))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(330, 80, 101, 61))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(440, 150, 101, 61))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(440, 80, 101, 61))
        self.pushButton_5.setObjectName("pushButton_5")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(700, 20, 281, 51))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(700, 80, 281, 51))  # 添加第二個標籤用來顯示角速度
        self.label_2.setObjectName("label_2")

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1018, 32))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "前"))
        self.pushButton_2.setText(_translate("MainWindow", "右"))
        self.pushButton_3.setText(_translate("MainWindow", "左"))
        self.pushButton_4.setText(_translate("MainWindow", "後"))
        self.pushButton_5.setText(_translate("MainWindow", "停止"))
        self.label.setText(_translate("MainWindow", "當前線速度: 0.00"))
        self.label_2.setText(_translate("MainWindow", "當前角速度: 0.00"))  # 初始化顯示速度

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.node = node

        # 連接按鈕到 ROS 2 功能
        self.ui.pushButton.clicked.connect(self.update_forward_speed)  # 前進按鈕
        self.ui.pushButton_2.clicked.connect(self.update_right_speed)   # 右轉按鈕
        self.ui.pushButton_3.clicked.connect(self.update_left_speed)    # 左轉按鈕
        self.ui.pushButton_4.clicked.connect(self.update_backward_speed) # 後退按鈕
        self.ui.pushButton_5.clicked.connect(self.update_stop_speed)     # 停止按鈕

    def update_forward_speed(self):
        linear_speed, angular_speed = self.node.forward()  # 獲取當前速度
        self.update_label(linear_speed, angular_speed)      # 更新標籤顯示速度

    def update_right_speed(self):
        linear_speed, angular_speed = self.node.right()  # 獲取當前速度
        self.update_label(linear_speed, angular_speed)

    def update_left_speed(self):
        linear_speed, angular_speed = self.node.left()  # 獲取當前速度
        self.update_label(linear_speed, angular_speed)

    def update_backward_speed(self):
        linear_speed, angular_speed = self.node.backward()  # 獲取當前速度
        self.update_label(linear_speed, angular_speed)

    def update_stop_speed(self):
        linear_speed, angular_speed = self.node.stop()  # 獲取當前速度
        self.update_label(linear_speed, angular_speed)

    def update_label(self, linear_speed, angular_speed):
        self.ui.label.setText(f"當前線速度: {linear_speed:.2f}")  # 更新顯示線速度
        self.ui.label_2.setText(f"當前角速度: {angular_speed:.2f}")  # 更新顯示角速度

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    
    # 確保 UI 顯示在啟動 ROS 之前
    node = MoveSpeedNode()
    main_window = MainWindow(node)
    main_window.show()

    # 在一個獨立的執行緒中運行 ROS 2 的 spin
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        sys.exit(app.exec_())
        executor_thread.exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
