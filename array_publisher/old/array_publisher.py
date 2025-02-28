import sys
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int16MultiArray, Int32MultiArray, Int64MultiArray, UInt8MultiArray, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray, Float32MultiArray, Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QSpinBox, QLabel, QComboBox, QTableWidget, QTableWidgetItem, QHBoxLayout, QHeaderView

class MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('array_publisher')
        self.publishers_dict = {}

        # GUI の設定
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("MultiArray Publisher")
        self.window.resize(800, 600)  # ウィンドウの初期サイズを大きく設定
        layout = QVBoxLayout()

        # 型の選択
        self.type_label = QLabel("Message Type:")
        layout.addWidget(self.type_label)
        self.type_selector = QComboBox()
        self.types = {
            "Int8MultiArray": Int8MultiArray,
            "Int16MultiArray": Int16MultiArray,
            "Int32MultiArray": Int32MultiArray,
            "Int64MultiArray": Int64MultiArray,
            "UInt8MultiArray": UInt8MultiArray,
            "UInt16MultiArray": UInt16MultiArray,
            "UInt32MultiArray": UInt32MultiArray,
            "UInt64MultiArray": UInt64MultiArray,
            "Float32MultiArray": Float32MultiArray,
            "Float64MultiArray": Float64MultiArray
        }
        self.type_selector.addItems(self.types.keys())
        layout.addWidget(self.type_selector)

        # トピック名設定
        self.topic_label = QLabel("Topic Name:")
        layout.addWidget(self.topic_label)
        self.topic_input = QLineEdit("multiarray_topic")
        layout.addWidget(self.topic_input)

        # データ数設定
        self.size_label = QLabel("Data Size:")
        layout.addWidget(self.size_label)
        self.size_input = QSpinBox()
        self.size_input.setMinimum(1)
        self.size_input.setMaximum(100)
        self.size_input.setValue(5)
        layout.addWidget(self.size_input)

        # トピック追加ボタン
        self.add_topic_button = QPushButton("Add Topic")
        self.add_topic_button.clicked.connect(self.add_topic)
        layout.addWidget(self.add_topic_button)

        # テーブルの作成
        self.topic_table = QTableWidget()
        self.topic_table.setColumnCount(4)
        self.topic_table.setHorizontalHeaderLabels(["Topic Name", "Message Type", "Data Size", "Data (comma separated)"])
        self.topic_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 列幅を自動調整
        self.topic_table.setMinimumWidth(750)  # テーブルの横幅を大きくする
        layout.addWidget(self.topic_table)

        # Publish 選択ボタン
        self.publish_selected_button = QPushButton("Publish Selected")
        self.publish_selected_button.clicked.connect(self.publish_selected_message)
        layout.addWidget(self.publish_selected_button)

        # Publish All ボタン
        self.publish_button = QPushButton("Publish All")
        self.publish_button.clicked.connect(self.publish_all_messages)
        layout.addWidget(self.publish_button)

        self.window.setLayout(layout)
        self.window.show()

    def add_topic(self):
        topic_name = self.topic_input.text()
        msg_type = self.types[self.type_selector.currentText()]
        data_size = self.size_input.value()

        if topic_name in self.publishers_dict:
            self.get_logger().error(f"Topic {topic_name} already exists!")
            return
        
        self.publishers_dict[topic_name] = {
            'publisher': self.create_publisher(msg_type, topic_name, 10),
            'type': msg_type,
            'data_size': data_size,
            'data': ""
        }
        
        row = self.topic_table.rowCount()
        self.topic_table.insertRow(row)
        self.topic_table.setItem(row, 0, QTableWidgetItem(topic_name))
        self.topic_table.setItem(row, 1, QTableWidgetItem(self.type_selector.currentText()))
        self.topic_table.setItem(row, 2, QTableWidgetItem(str(data_size)))
        self.topic_table.setItem(row, 3, QTableWidgetItem(""))
        self.get_logger().info(f"Added topic {topic_name}")

    def publish_selected_message(self):
        selected_row = self.topic_table.currentRow()
        if selected_row >= 0:
            topic_name = self.topic_table.item(selected_row, 0).text()
            self.publish_message(topic_name)

    def publish_all_messages(self):
        for row in range(self.topic_table.rowCount()):
            topic_name = self.topic_table.item(row, 0).text()
            self.publish_message(topic_name)

    def publish_message(self, topic_name):
        info = self.publishers_dict[topic_name]
        row = [self.topic_table.item(i, 0).text() for i in range(self.topic_table.rowCount())].index(topic_name)
        data_text = self.topic_table.item(row, 3).text()
        
        try:
            data_list = list(map(float if 'Float' in info['type'].__name__ else int, data_text.split(',')))
            if len(data_list) != info['data_size']:
                self.get_logger().error(f"Data size mismatch for {topic_name}!")
                return
        except ValueError:
            self.get_logger().error(f"Invalid data input for {topic_name}. Must be numbers separated by commas.")
            return

        msg = info['type']()
        msg.data = data_list
        info['publisher'].publish(msg)
        self.get_logger().info(f"Published to {topic_name}: {msg.data}")

    def run(self):
        sys.exit(self.app.exec_())


def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayPublisher()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
