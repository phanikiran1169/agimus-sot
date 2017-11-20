from dynamic_graph_hpp.sot import RosQueuedSubscribe
from dynamic_graph_bridge_msgs.msg import Vector
import rospy, time

pub = rospy.Publisher ('topic', Vector, queue_size=10)
rospy.init_node('talker', anonymous=True)

rqe = RosQueuedSubscribe ('rqe')
rqe.add ('vector', 'sig', 'topic')

time.sleep(1)

N = 1

s = rqe.sig
s.recompute (s.time + 1)
# print s.value

pub.publish (Vector([0] * N))
time.sleep(1)
print s.value

s.recompute (s.time + 1)
print s.value

for i in range (10):
    pub.publish (Vector([i] * N))
    time.sleep(0.001)

for i in range (10):
    s.recompute (s.time + 1)
    print s.value
