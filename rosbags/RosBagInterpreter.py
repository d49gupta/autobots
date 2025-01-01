import rosbag 
import bagpy
from bagpy import bagreader
import pandas as pd

# bag = rosbag.Bag('MH_01_easy.bag')
# for topic, msg, t in bag.read_messages(topics=['/cam0/image_raw', '/cam1/image_raw', '/imu0', '/leica/position']):
#     print(msg)
# bag.close()

bag = bagreader('MH_01_easy.bag')
imu_msg = bag.message_by_topic('/imu0')
print(imu_msg)