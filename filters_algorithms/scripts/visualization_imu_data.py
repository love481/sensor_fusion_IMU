import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32
import tf
import time
filtered_t=[]
filtered_yaw=[]
raw_yaw=[]
raw_t=[]
plt.ion()
figure, ax = plt.subplots()
ax.set_title('yaw_plot')
ax.axis([0,100000,-180,180])
class IMU_visualize:
    def __init__(self):
        rospy.init_node('imu_visualize', anonymous=True)
        rospy.Subscriber("IMU/Raw_yaw",Float32, self.Raw_callback)
        rospy.Subscriber("IMU/yaw",Float32, self.Filtered_callback)
        self.prevTime=rospy.Time().now()
        self.fdt=0
        self.rdt=0
    def Raw_callback(self,data):
            #rospy.loginfo("%lf",self.data.data)
            raw_yaw.append(data.data)
            self.rdt=self.rdt+(rospy.Time().now()-self.prevTime).to_sec()
            raw_t.append(self.rdt)
    def Filtered_callback(self,data):
            #rospy.loginfo("%lf",self.curr.data)
            filtered_yaw.append(data.data)
            self.fdt=self.fdt+(rospy.Time().now()-self.prevTime).to_sec()
            filtered_t.append(self.fdt)
    def run(self):
        while not rospy.is_shutdown():
            #plt.plot([self.curr.x,self.prev.x],[self.curr.y,self.prev.y],'r',linewidth=1)
            plt.plot(filtered_t,filtered_yaw,'-b',linewidth=0.9)
            plt.plot(raw_t,raw_yaw,'-r',linewidth=0.9)
            figure.canvas.draw()
            figure.canvas.flush_events()
            time.sleep(0.01)

if __name__=="__main__":
    try:
        IMU_visualize_=IMU_visualize()
        IMU_visualize_.run()
    except rospy.ROSInterruptException:
        pass
