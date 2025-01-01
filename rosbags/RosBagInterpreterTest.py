import rosbag 
# import bagpy
# from bagpy import bagreader
# import pandas as pd

bag = rosbag.Bag('MH_01_easy.bag')
topics = ['/cam0/image_raw', '/cam1/image_raw', '/imu0', '/leica/position']
for topic, msg, t in bag.read_messages(topics=['/imu0']):
    print(msg)
    break
bag.close()

# bag = bagreader('MH_01_easy.bag')
# imu_msg = bag.message_by_topic('/leica/position')
# print(imu_msg)