#! /usr/bin/python

# tutorial from https://www.tutorialspoint.com/python/python_gui_programming.htm


import Tkinter as tk
import rospy 
from ros_end_effector.msg import EEGraspControl, EEPinchControl
    
def pubMessage(pub, msg, seq, percentage, finger_1Name=None, finger_2Name=None):
    
    if  isinstance(msg, EEPinchControl) :
        msg.finger_pinch_1 = finger_1Name
        msg.finger_pinch_2 = finger_2Name

    msg.seq = seq
    seq += 1
    msg.stamp = rospy.Time.now()
    msg.percentage = percentage
    rospy.loginfo("Publishing Grasp message: ")
    rospy.loginfo(msg)
    pub.publish(msg)


if __name__ == "__main__":
    
    #ros things
    pubGrasp = rospy.Publisher("ros_end_effector/grasp", EEGraspControl, queue_size=1)
    pubPinch = rospy.Publisher("ros_end_effector/pinch", EEPinchControl, queue_size=1)
    rospy.init_node('gui_pub', anonymous=True)

    #gui things
    winlength = 500
    winheight = 200
    winposition = "800+500"
    window = tk.Tk()
    window.title("ros_end_effector control")
    window.geometry(str(winlength) + "x" + str(winheight) + "+" + winposition)
    
    #grid layout for multiple widgets
    topRow = tk.Frame(window)
    topRow.grid(column=0, row=0)

    bottomRow = tk.Frame(window)
    bottomRow.grid(column=0, row=1)

    scaleGraspValue = tk.DoubleVar()
    scaleGrasp = tk.Scale( topRow, 
                    variable = scaleGraspValue, 
                    orient=tk.HORIZONTAL, 
                    length=winlength-100,
                    from_=0, to=1,
                    showvalue=1, 
                    tickinterval=0.1, resolution=0.01,
                    label="Open (0) Close(1) GRASP")
    scaleGrasp.pack(side=tk.LEFT, padx=10, pady=0)

    scalePinchValue = tk.DoubleVar()
    scalePinch = tk.Scale( bottomRow, 
                    variable = scalePinchValue, 
                    orient=tk.HORIZONTAL, 
                    length=winlength-100,
                    from_=0, to=1,
                    showvalue=1,
                    tickinterval=0.1, resolution=0.01,
                    label="Open (0) Close(1) PINCH")
    scalePinch.pack(side=tk.LEFT, padx=10, pady=0)


    #loop
    rate = rospy.Rate(10) # 10hz
    msgGrasp = EEGraspControl()
    msgPinch = EEPinchControl()
    seqGrasp = 0 #sequence incrementing for msg
    seqPinch = 0

    while not rospy.is_shutdown():
        #update gui tkinter values
        window.update_idletasks()
        window.update()
        
        pubMessage(pubGrasp, msgGrasp, seqGrasp, float(scaleGraspValue.get()) )
        seqGrasp += 1
        
        #now only one can be published otherwise they will be in conflict
        #pubMessage(pubPinch, msgPinch, seqPinch, float(scalePinchValue.get()), "finger_1", "finger_2" )                
        #seqPinch += 1
        scalePinch['state'] = 'disabled'

        rate.sleep()


