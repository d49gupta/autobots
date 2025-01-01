from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

bag_path = 'MH_01_easy.bag'
topic = '/cam0/image_raw'
typestore = get_typestore(Stores.ROS1_NOETIC)
dataDict = {}

with Reader(bag_path) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic:
            try:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                if connection.topic not in dataDict:
                    dataDict[connection.topic] = []
                dataDict[connection.topic].append({'timestamp': timestamp, 'message': msg})
                print(f"Timestamp: {timestamp}, Topic: {connection.topic}, Message: {msg}")
            except Exception as e:
                print(f"Error deserializing message: {e}")

print(dataDict.keys())