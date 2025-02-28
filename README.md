# MultiArray Publisher for ROS 2

This is a GUI tool for publishing various MultiArray message types in ROS 2. It allows users to define topics dynamically, set data sizes, and specify values for publishing.

## Features

- Supports multiple MultiArray message types:

  - 'Int8MultiArray', 'Int16MultiArray', 'Int32MultiArray', 'Int64MultiArray'

  - 'UInt8MultiArray', 'UInt16MultiArray', 'UInt32MultiArray', 'UInt64MultiArray'

  - 'Float32MultiArray', 'Float64MultiArray'

- Add topics dynamically with:

  - Topic Name

  - Message Type

  - Data Size

- Edit data after adding a topic

- Publish messages for selected topics or all topics

- Delete selected topics

## Prerequisites

- ROS 2

- Python 3

- rclpy

- PyQt5

## Installation

1. Install ROS 2 and set up your environment:

   'source /opt/ros/your_ros2_distro/setup.bash'

2. Install necessary Python dependencies:

   'pip install rclpy PyQt5'

3. Clone the repository:
   'cd /your/ws/directory'
   
   'git clone github.com/tgkhtknk/array-publish-ros2'
   
   'colcon build --packages-select array_publisher'

   'source install/setup.bash'

## Usage

Run the script to start the GUI:

ros2 run array_publisher array_publihser

### GUI Overview

1. Select Message Type - Choose the type of MultiArray message.

2. Enter Topic Name - Specify the ROS 2 topic name.

3. Set Data Size - Define the number of elements in the array.

4. Click "Add Topic" - Add a new topic to the table.

5. Edit Data - Modify the comma-separated values in the data field.

6. Publish Messages:

   - Click "Publish Selected" to publish a message for a selected topic.

   - Click "Publish All" to publish all topics.

7. Delete Topics - Select a topic and click "Delete Selected Topic" to remove it.

## License

This project is licensed under the MIT License.

## Contributing

Feel free to open issues or submit pull requests to improve this tool.

