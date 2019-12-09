#! /usr/bin/python

# tutorial from https://www.tutorialspoint.com/python/python_gui_programming.htm


import Tkinter as tk
import rospy 
from ros_end_effector.msg import EEGraspControl

def scale_clb(v):
    l.config(text='you have selected ' + v)
    scaleValue = v

#ros things
pub = rospy.Publisher("chatter", EEGraspControl, queue_size=10)
rospy.init_node('gui_pub', anonymous=True)

#gui things
winlength = 500
winheight = 200
winposition = "800+500"

window = tk.Tk()
window.title("ros_end_effector control")
window.geometry(str(winlength) + "x" + str(winheight) + "+" + winposition)

l = tk.Label(window, bg='white', fg='black', width=100, text='empty')
l.pack()

scaleValue = tk.DoubleVar()
scale = tk.Scale( window, 
                 variable = scaleValue, 
                 orient=tk.HORIZONTAL, 
                 length=winlength,
                 showvalue=0,tickinterval=10, resolution=1,
                 command=scale_clb,
                 label="Open (0) Close(100) GRASP")

scale.pack(anchor=tk.CENTER)

#loop
rate = rospy.Rate(10) # 10hz
msg = EEGraspControl()
seq = 0 #sequence incrementing for msg
timestamp = 0 #TODO get real timestamp

while not rospy.is_shutdown():
    #update gui tkinter values
    window.update_idletasks()
    window.update()
    
    msg.seq = seq
    seq += 1
    msg.stamp = timestamp
    print (type(scaleValue))
    msg.percentage = float(scaleValue.get())/100
    rospy.loginfo("Publishing message: ")
    rospy.loginfo(msg)
    pub.publish(msg)
    
    rate.sleep()


