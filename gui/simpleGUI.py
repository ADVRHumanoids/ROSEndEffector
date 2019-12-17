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
        winheight = 400
        winposition = "800+500"
        
        self.rootWin = tk.Tk()
        self.rootWin.title("ros_end_effector control GUI")
        self.rootWin.geometry(str(winlength) + "x" + str(winheight) + "+" + winposition)

        
        #create the scale widgets
        self.scaleGrasp = GuiScale(self.createFrameinGrid(0,0), 
                                   "ros_end_effector/grasp", EEGraspControl)
        
        
        #create checkbuttons for the pinch scale
        #TODO take finger names from urdf

        maxPicks = 2 #only 2 finger for the pinch
        self.scalePinch = GuiScaleCheck(self.createFrameinGrid(0,1), 
                                   "ros_end_effector/pinch", EEPinchControl,
                                   ["finger_1", "finger_2", "finger_3"], maxPicks)

    def createFrameinGrid(self, col, row):
        """
        Create a new frame and put it into rootWindow with grid(col, row)
        """
        widgetRow = tk.Frame(self.rootWin)
        widgetRow.grid(column=col, row=row, padx =0, pady=15)
        return widgetRow
            

        
        
class GuiScale:
    def __init__(self, frame, topicName, msgType):
        self.value = tk.DoubleVar()
        self.scale = tk.Scale(frame,
            variable = self.value,
            command = self.scaleClbk,
            orient = tk.HORIZONTAL, 
            length = 400,
            from_ = 0, to = 1,
            showvalue = 0, #default is 1
            tickinterval = 0.1, resolution = 0.01,
            label = topicName)
        self.scale.grid(row=0, column=0, padx=0, pady=0)
        
        #the entry widget to enter percentage directly
        entryVcmd = (frame.register(self.entryClbk))       
        self.entry = tk.Entry(frame,
                              width = 4,
                              textvariable = self.value,
                              validate='focusout',
                              validatecommand = (entryVcmd, '%P'))
        self.entry.grid(row=0, column=1, padx=1, pady=0)
        
        #for ros messages
        self.msgType = msgType
        self.pub = rospy.Publisher(topicName, msgType, queue_size=1)
        self.msgSeq = 0 #sequence incrementing for msg
        
        
    def scaleClbk ( self, value ):
        msg = self.msgType()
        msg.seq = self.msgSeq
        self.msgSeq += 1
        msg.stamp = rospy.Time.now()
        msg.percentage = float(value)
        rospy.loginfo("Publishing Grasp message: ")
        rospy.loginfo(msg)
        self.pub.publish(msg)
        
    def entryClbk ( self, P ):
        if str.isdigit(P):
            self.scaleClbk(P)
            return True
        else:
            return False
        

class GuiScaleCheck:
    """
    Scale widget associated with some checkbuttons (useful for pinch command)
    """
    def __init__(self, frame, topicName, msgType, fingerList, maxPicks):
        
        self.selectedFinger = ["", ""]
        
        #checkbuttons part
        self.checkButtons = []
        self.varCheckButtons = []
        self.maxPicks = maxPicks
        
        for idx, finger in enumerate(fingerList):
            self.varCheckButtons.append(tk.IntVar())
            self.checkButtons.append(tk.Checkbutton(frame, 
                                        text = finger,
                                        variable = self.varCheckButtons[idx],
                                        command = self.check_picked))
            self.checkButtons[idx].grid(row=0, column=idx, padx = 0, pady=0)
            
        #scale part
        self.scaleVar = tk.DoubleVar()
        self.scale = tk.Scale(frame,
            variable = self.scaleVar,
            command = self.scaleClbk,
            orient = tk.HORIZONTAL, 
            length = 400,
            from_ = 0, to = 1,
            showvalue = 0, 
            tickinterval = 0.1, resolution = 0.01,
            label = topicName,
            state = tk.DISABLED)
        self.scale.grid(row=1, column=0, columnspan=len(fingerList)-1, padx=0, pady=0)
        
        #the entry widget to enter percentage directly
        entryVcmd = (frame.register(self.entryClbk))       
        self.entry = tk.Entry(frame,
                              width = 4,
                              textvariable = self.scaleVar,
                              validate='focusout',
                              validatecommand = (entryVcmd, '%P'),
                              state = tk.DISABLED)
        self.entry.grid(row = 1, column = len(fingerList)-1 , padx=1, pady=0)
        
        #for ros messages
        self.msgType = msgType
        self.pub = rospy.Publisher(topicName, msgType, queue_size=1)
        self.msgSeq = 0 #sequence incrementing for msg
        
        
    def entryClbk ( self, P ):
        if str.isdigit(P):
            self.scaleClbk(P)
            return True
        else:
            return False
        
    def scaleClbk(self, value):
        msg = self.msgType()
        msg.seq = self.msgSeq
        self.msgSeq += 1
        msg.stamp = rospy.Time.now()
        msg.percentage = float(value)
        msg.finger_pinch_1 = self.selectedFinger[0]
        msg.finger_pinch_2 = self.selectedFinger[1]
        rospy.loginfo("Publishing Pinch message: ")
        rospy.loginfo(msg)
        self.pub.publish(msg)
       
       
    def check_picked(self) :
        #each selected checkbutton has a var == 1
        nPick = sum(var.get() for var in self.varCheckButtons)
        
        if self.maxPicks == nPick:
            nSelect = 0
            for chk, var in zip(self.checkButtons, self.varCheckButtons):
                if var.get():
                    self.selectedFinger[nSelect] = chk['text']
                    nSelect += 1
                else: #if zero, button has not been selected, disable it
                    chk['state'] = tk.DISABLED
            
            #enable scale and relative entry
            self.scale['state'] = tk.NORMAL
            self.entry['state'] = tk.NORMAL
        
        else:
        #enable all buttons and disable scale
            self.scale['state'] = tk.DISABLED
            self.entry['state'] = tk.DISABLED

            for chk in self.checkButtons:
                chk['state'] = tk.NORMAL





if __name__ == "__main__":
    
    #ros things
    rospy.init_node('gui_pub', anonymous=True)
    
    #gui things
    gui = Gui()

    gui.rootWin.mainloop()




