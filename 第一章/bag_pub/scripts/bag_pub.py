import rosbag
import os
def rosbag_merge(bag_merge,path):
    bagn = rosbag.Bag(path,'r') #读取到的包
    # bagn = rosbag.Bag('/home/ubuntu/catkin_ws/n.bag','r') #可指定包信息


    #给定时间段，根据自己需要更改就行
    t1 = 1638832875       #开始时间，单位秒
    t2 = 1638832875 + 100  #结束时间，单位秒

    #包操作： 时间戳+话题过滤
    for topic, msg, t in bagn:      # 注意topic、msg、t顺序，从包中解析依次是话题、该话题消息内容、该话题的时间戳，推荐使用所示顺序
        tmp = t
        tmp = float(str(tmp)) / 1e+9 # 转化为秒
        if float(t1) <= tmp <= float(t2):
            if topic == '/topic_1':
                bag_merge.write(topic, msg, t)
            #如果有其他话题需求，依次叠加,如
            if topic == '/topic_2':
                bag_merge.write(topic, msg, t)

    
    #包源n： 时间戳  /  时间戳+话题，基本操作样式如上。当然也是可以添加话题类型过滤等等，根据实际需求进行调整即可
    bagn.close()


if __name__== "__main__":
    bag_merge = rosbag.Bag('/home/ubuntu/catkin_ws/merge.bag','w')  #新包merge.bag
    # 读取当前bag包路径
    path = os.path.split(os.path.realpath(__file__))[0]
    # 判断当前目录下是否存在bag包
    for files in os.listdir(path):
        if files.endswith(('bag')):
            bag_path = os.path.join(path, files)
            rosbag_merge(bag_merge,path)
    bag_merge.close() #注意，所有打开过的包，都得关闭，否则下次访问可能会失败