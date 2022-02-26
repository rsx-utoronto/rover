import rosbag
new_bag  = rosbag.Bag('fused.bag', 'w')
bags = ['ins_imu.bag', '2022-02-19-22-37-59.bag', '2022-02-20-23-21-26.bag']
for bag_file in bags:
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['ins', 'fix', 'imu', 'mag']):
        new_bag.write(topic, msg, t)
    bag.close()
new_bag.close()