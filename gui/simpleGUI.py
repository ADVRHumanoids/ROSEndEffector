#! /usr/bin/python

import Tkinter as tk
import rospy 
from ros_end_effector.msg import EEGraspControl, EEPinchControl

class Gui:
    """
    Main gui window which creates all the widget inside it
    """
    def __init__(self):
        
        #windows params and style TODO make it nice 
        winlength = 500
        winheight = 200
        winposition = "800+500"
        
        self.rootWin = tk.Tk()
        self.rootWin.title("ros_end_effector control GUI")
        self.rootWin.geometry(str(winlength) + "x" + str(winheight) + "+" + winposition)
        
        #grid layout for multiple widgets
        topRow = tk.Frame(self.rootWin)
        topRow.grid(column=0, row=0)
        bottomRow = tk.Frame(self.rootWin)
        bottomRow.grid(column=0, row=1)
        
        #create the scale widgets
        #TODO create widgets in a list of objects
        self.scaleGrasp = GuiScale(topRow, "ros_end_effector/grasp", EEGraspControl)
        #now only one can be active otherwise they will be in conflict
        #self.scalePinch = GuiScale(bottomRow, "Open (0) Close(1) PINCH")

        
        
class GuiScale:
    def __init__(self, frame, topicName, msgType):
        self.value = tk.DoubleVar()
        self.updated = False
        self.scale = tk.Scale(frame,
            variable = self.value,
            command = self.scaleClbk,
            orient = tk.HORIZONTAL, 
            length = 400,
            from_ = 0, to = 1,
            showvalue = 1, 
            tickinterval = 0.1, resolution = 0.01,
            label = topicName)
        self.scale.pack(side=tk.LEFT, padx=10, pady=0)
        
        #for ros messages
        self.msgType = msgType
        self.pub = rospy.Publisher(topicName, msgType, queue_size=1)
        self.msgSeq = 0 #sequence incrementing for msg

        
        
    def scaleClbk(self, value):
        #TODO if pinch... different type of msg, switch somewhere?
        msg = self.msgType()
        msg.seq = self.msgSeq
        self.msgSeq += 1
        msg.stamp = rospy.Time.now()
        print (value)
        msg.percentage = float(value)
        rospy.loginfo("Publishing Grasp message: ")
        rospy.loginfo(msg)
        self.pub.publish(msg)


if __name__ == "__main__":
    
    #ros things
    rospy.init_node('gui_pub', anonymous=True)
    
    #gui things
    gui = Gui()

    gui.rootWin.mainloop()




