import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32
import tf
import time
t=[]
yaw=[]
Ryaw=[]
rt=[]
plt.ion()
figure, ax = plt.subplots()
ax.set_title('yaw_plot')
ax.axis([0,100000,-180,180])
class IMU_visualize:
    def __init__(self):
        rospy.init_node('imu_visualize', anonymous=True)
        rospy.Subscriber("IMU/yaw",Float32, self.callback)
        rospy.Subscriber("IMU/Raw_yaw",Float32, self.callbackR)
        self.prev=Float32()
        self.curr=Float32()
        self.prev.data=0
        self.curr.data=0
        self.prevTime=rospy.Time().now()
        self.dt=0
        self.rdt=0
    def callback(self,data):
            #rospy.loginfo("%lf",self.curr.data)
            self.curr.data=data.data
            yaw.append(self.curr.data)
            self.dt=self.dt+(rospy.Time().now()-self.prevTime).to_sec()
            t.append(self.dt)
    def callbackR(self,data):
            #rospy.loginfo("%lf",self.curr.data)
            self.prev.data=data.data
            Ryaw.append(self.prev.data)
            self.rdt=self.rdt+(rospy.Time().now()-self.prevTime).to_sec()
            rt.append(self.rdt)
    def run(self):
        while not rospy.is_shutdown():
            #plt.plot([self.curr.x,self.prev.x],[self.curr.y,self.prev.y],'r',linewidth=1)
            plt.plot(t,yaw,'-b',linewidth=0.9)
            plt.plot(rt,Ryaw,'-r',linewidth=0.9)
            figure.canvas.draw()
            figure.canvas.flush_events()
            time.sleep(0.01)

if __name__=="__main__":
    try:
        IMU_visualize_=IMU_visualize()
        IMU_visualize_.run()
    except rospy.ROSInterruptException:
        pass
