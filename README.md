#MultiArray Publisher for ROS 2

This is a GUI tool for publishing various MultiArray message types in ROS 2. It allows users to define topics dynamically, set data sizes, and specify values for publishing.

##Features

Supports multiple MultiArray message types:

Int8MultiArray, Int16MultiArray, Int32MultiArray, Int64MultiArray

UInt8MultiArray, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray

Float32MultiArray, Float64MultiArray

Add topics dynamically with:

Topic Name

Message Type

Data Size

Edit data after adding a topic

Publish messages for selected topics or all topics

Delete selected topics

##Prerequisites

ROS 2 (Humble or compatible version)

Python 3

rclpy

PyQt5

##Installation

Install ROS 2 and set up your environment:

source /opt/ros/humble/setup.bash

Install necessary Python dependencies:

pip install rclpy PyQt5

Clone the repository:

git clone <repository_url>
cd <repository_name>

##Usage

Run the script to start the GUI:

python multiarray_publisher.py

GUI Overview

Select Message Type - Choose the type of MultiArray message.

Enter Topic Name - Specify the ROS 2 topic name.

Set Data Size - Define the number of elements in the array.

Click "Add Topic" - Add a new topic to the table.

Edit Data - Modify the comma-separated values in the data field.

Publish Messages:

Click "Publish Selected" to publish a message for a selected topic.

Click "Publish All" to publish all topics.

Delete Topics - Select a topic and click "Delete Selected Topic" to remove it.

##License

This project is licensed under the MIT License.

##Contributing

Feel free to open issues or submit pull requests to improve this tool.

