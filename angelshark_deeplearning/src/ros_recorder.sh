# This file create nested folders and uses rosbag to record the drive

mkdir -p ~/Desktop/DataPacket/RosbagRecords
rosbag record -o ~/Desktop/DataPacket/RosbagRecords/angshrk /angelshark/ackermann_cmd /angelshark/camera/image_raw /angelshark/imu /angelshark/scan