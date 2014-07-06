#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('/slam/occupancyGridMap', OccupancyGrid , queue_size=10)
    rospy.init_node('talker', anonymous = False)
    r = rospy.Rate(1) # 1 hz
    my_map = create_simple_map()
    while not rospy.is_shutdown():
       
        rospy.loginfo("%Publishing a Simple Map")
        
        pub.publish(my_map)
        r.sleep()
        
        
def create_simple_map():
  
  simple_map = OccupancyGrid()
  
  simple_map.info.width = 200
  simple_map.info.height = 200
  simple_map.data = [ ]
  simple_map.data.append(3)
  for i in range(10):
      for j in range(10):
          if (i == 0 or i ==50 or i ==100):
              simple_map.data.append(126)
          else :
              simple_map.data.append(50)
      
  return simple_map
      

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
