import rospy
import threading
import tkinter as tk
import tkinter.font as tkfont
from PIL import Image, ImageTk
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class control_node(object):
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.ps = PoseStamped()

        # # Altitude setpoint, [meters]
        self.ALT_SP = 0
        # update the setpoint message with the required altitude
        self.ps.pose.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 10.0

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print("offboard")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)
	# Callbacks
    # local position callback
    def pos_cb(self, msg):
        self.local_pos_x = round(msg.pose.position.x,1)
        self.local_pos_y = round(msg.pose.position.y,1)
        self.local_pos_z = round(msg.pose.position.z,1)
        self.local_quat_x = round(msg.pose.orientation.x,1)
        self.local_quat_y = round(msg.pose.orientation.y,1)
        self.local_quat_z = round(msg.pose.orientation.z,1)
        self.local_quat_w = round(msg.pose.orientation.w,1)

    # Drone State callback
    def state_cb(self, msg):
        self.state = msg

    # Update setpoint message
    def hold(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("Hold")
        self.var.set("Hold the current position")
        self.st = "X"

    def takeoff(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = 2
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("takeoff")
        self.var.set("Takeoff to the hight of 2 meters")
        self.st="T"
        if (self.local_pos_z >= 2):
            self.var.set("Takeoff successfully")

    def x_right(self):
        self.ps.pose.position.x = (self.local_pos_x + 0.5)
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Move to the right")
        print("right")
        self.st="R"

    def x_left(self):
        self.ps.pose.position.x = (self.local_pos_x - 0.5)
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("left")
        self.var.set("Move to the left")
        self.st="L"

    def y_forward(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = (self.local_pos_y + 0.5)
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("forward")
        self.var.set("Move forward")
        self.st="F"

    def y_back(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = (self.local_pos_y - 0.5)
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Move backward")
        print("backward")
        self.st="B"

    def landing(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = 0
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Landing")
        print("landing")
        self.st="D"
        if (self.local_pos_z <= 0):
            self.st= "E"
            
            if not self.state.MODE_PX4_LAND:
                self.setAutoLandMode()
                self.rate.sleep()
                self.setDisarm()
                self.rate.sleep()
            else:
                print("Exit")
    
    def org(self):
        self.ps.pose.position.x = 0
        self.ps.pose.position.y = 0
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = 0
        self.ps.pose.orientation.y = 0
        self.ps.pose.orientation.z = 0
        self.ps.pose.orientation.w = self.local_quat_w
        print("(0,0)")
        self.var.set("Move to origin")
        self.st= "O"

    def z_right(self):
        self.ps.pose.position.x = self.local_pos_x 
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = -0.2
        self.ps.pose.orientation.w = self.local_quat_w
        print("rotate right")
        self.var.set("rotate right")
        self.st="A"

    def z_left(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = 0.2
        self.ps.pose.orientation.w = self.local_quat_w
        print("rotate left")
        self.var.set("rotate left")
        self.st="S"

    def z_up(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = (self.local_pos_z + 0.5)
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("up")
        self.var.set("Move upward")
        self.st= "W"

    def z_down(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = (self.local_pos_z - 0.5)
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        print("Down")
        self.var.set("Move downward")
        self.st= "N"

    def reinit(self):
        self.st= "I"
        self.setArm()
        self.sp_pub.publish(self.ps)
        # self.var.set("initiating")
        print("init setpoint")

    def z_rotation(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = (self.local_quat_z - 0.1)
        self.ps.pose.orientation.w = self.local_quat_w
        print("Rotation")
        self.var.set("Rotation")
        self.st= "Q"

    def setpos(self):
        setx = self.xentry.get()
        sety = self.yentry.get()
        setz = self.zentry.get()
        setrz = self.rzentry.get()
        if setx == "" or sety == "" or setz == "" or setrz == "":
            self.ps.pose.position.x = self.local_pos_x
            self.ps.pose.position.y = self.local_pos_y
            self.ps.pose.position.z = self.local_pos_z
            self.ps.pose.orientation.z = self.local_quat_z
        else:
            self.ps.pose.position.x = float(setx)
            self.ps.pose.position.y = float(sety)
            self.ps.pose.position.z = float(setz)
            self.ps.pose.orientation.z = float(setrz)
            
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.w = self.local_quat_w
        self.var.set("Setting Position")
        self.st= "P"       

    def clear(self):
        self.xentry.delete(0, 'end')
        self.yentry.delete(0, 'end')
        self.zentry.delete(0, 'end')

    def auto(self):
        self.st= "U"
        self.modes = "auto"
        self.takeoffbt["state"] = "disabled"  
        self.leftbt["state"] = "disabled"
        self.rightbt["state"] = "disabled"
        self.backwardbt["state"] = "disabled"  
        self.forwardbt["state"] = "disabled"
        self.downbt["state"] = "disabled"
        self.upbt["state"] = "disabled" 
        self.rrightbt["state"] = "disabled"
        self.rleftbt["state"] = "disabled"
        # self.robt["state"] = "disabled"
        self.stopbt["state"] = "disabled"
        # self.orgbt["state"] = "disabled" 
        self.var.set("Auto loop")
        if (self.local_pos_z < 1.8):
            self.ps.pose.position.x = self.local_pos_x
            self.ps.pose.position.y = self.local_pos_y
            self.ps.pose.position.z = 2
            self.sp_pub.publish(self.ps)
        elif (self.local_pos_z >= 2)&(self.local_pos_x <= 1)&(self.local_pos_y <= 0.2):
            self.ps.pose.position.x = (self.local_pos_x + 0.5)
            self.ps.pose.position.y = self.local_pos_y
            self.ps.pose.position.z = self.local_pos_z
            self.sp_pub.publish(self.ps)
        elif (self.local_pos_z >= 2)&(self.local_pos_x >= 1)&(self.local_pos_y <= 1):
            self.ps.pose.position.x = self.local_pos_x
            self.ps.pose.position.y = (self.local_pos_y + 0.5)
            self.ps.pose.position.z = self.local_pos_z
            self.sp_pub.publish(self.ps)
        elif (self.local_pos_z >= 1)&(self.local_pos_x >= 0)&(self.local_pos_y >= 0.9):
            self.ps.pose.position.x = (self.local_pos_x - 0.5)
            self.ps.pose.position.y = self.local_pos_y
            self.ps.pose.position.z = self.local_pos_z
            self.sp_pub.publish(self.ps)   
        elif (self.local_pos_z >= 1)&(self.local_pos_x <= 0)&(self.local_pos_y >= 0):
            self.ps.pose.position.x = self.local_pos_x
            self.ps.pose.position.y = (self.local_pos_y - 0.5)
            self.ps.pose.position.z = self.local_pos_z
            self.sp_pub.publish(self.ps)   
        elif (self.local_pos_z >= 1)&(self.local_pos_x == 0)&(self.local_pos_y == 0): 
                self.landing() 
                self.var.set("Landing")  
                self.sp_pub.publish(self.ps)

    def manual(self):
        self.ps.pose.position.x = self.local_pos_x
        self.ps.pose.position.y = self.local_pos_y
        self.ps.pose.position.z = self.local_pos_z
        self.ps.pose.orientation.x = self.local_quat_x 
        self.ps.pose.orientation.y = self.local_quat_y
        self.ps.pose.orientation.z = self.local_quat_z
        self.ps.pose.orientation.w = self.local_quat_w
        self.modes = "manual"
        self.st = "M" 
        self.takeoffbt["state"] = "normal"  
        self.leftbt["state"] = "normal" 
        self.rightbt["state"] = "normal"  
        self.backwardbt["state"] = "normal"  
        self.forwardbt["state"] = "normal"  
        self.downbt["state"] = "normal"
        self.upbt["state"] = "normal"  
        self.rrightbt["state"] = "normal"
        self.rleftbt["state"] = "normal" 
        # self.robt["state"] = "normal"  
        self.stopbt["state"] = "normal" 
        # self.orgbt["state"] = "normal" 

    def gui(self):

        root = tk.Tk()
        root.title("Control GUI")

        #title 
        titlef = tk.Frame(root)
        menubar = tk.Menu(root)
        menuList = tk.Menu(menubar, tearoff=0)
        menuList.add_command(label="Exit", command=lambda:root.destroy())
        menubar.add_cascade(label="Menu", menu=menuList)
        root.config(menu=menubar)

        #title style
        title_font = tkfont.Font(family='Helvetica', size=24, weight="bold", slant="italic")

        #title body
        title = tk.Label(titlef, text="Control GUI", font=title_font, anchor="e" )
        title.grid(row=0, column=1,sticky="")
        titlef.grid(row=0, column=0, columnspan=5,sticky="")

        #Control modes
        conMode = tk.LabelFrame(root, text="Control Modes", width=200)
        conMode.grid(row=2, column=0, columnspan=5,sticky="W")

        offbt = tk.Button(conMode, text="Manual", width=10,bd=2, cursor="exchange", command = lambda: self.manual())
        offbt.grid(row=0, column=1, columnspan=1)

        autobt = tk.Button(conMode, text="Auto", width=10,bd=2, cursor="exchange", command = lambda: self.auto())
        autobt.grid(row=0, column=0, columnspan=1)

        #manualState
        manualSate = tk.LabelFrame(root, text="manualState", width=200)
        manualSate.grid(row=3, column=0, columnspan=5,sticky="W")

        timg = Image.open('Icon/Takeoff.jpg')
        tics = timg.resize((30, 30))
        takeic = ImageTk.PhotoImage(tics)
        self.takeoffbt = tk.Button(manualSate,image = takeic, text="Takeoff", width=100,bd=2, compound="left", cursor="exchange", command = lambda: self.takeoff(),state="disabled")
        self.takeoffbt.pack()
        self.takeoffbt.grid(row=0, column=0, columnspan=1)
        
        limg = Image.open('Icon/Land.jpg')
        lics = limg.resize((30, 30))
        landic = ImageTk.PhotoImage(lics)
        landbt = tk.Button(manualSate, text="Landing",image = landic, width=100,bd=2, compound="left", cursor="exchange", command = lambda: self.landing())
        landbt.grid(row=0, column=1, columnspan=1)

        fimg = Image.open('Icon/For.jpg')
        fics = fimg.resize((30, 30))
        foric = ImageTk.PhotoImage(fics)
        self.forwardbt = tk.Button(manualSate, text="Forword", image = foric, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.y_forward(),state="disabled")
        self.forwardbt.grid(row=1, column=0, columnspan=1)

        bimg = Image.open('Icon/Back.jpg')
        bics = bimg.resize((30, 30))
        backic = ImageTk.PhotoImage(bics)
        self.backwardbt = tk.Button(manualSate, text="Backward", image = backic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.y_back(),state="disabled")
        self.backwardbt.grid(row=1, column=1, columnspan=1)

        rimg = Image.open('Icon/right.jpg')
        rics = rimg.resize((30, 30))
        rightic = ImageTk.PhotoImage(rics)
        self.rightbt = tk.Button(manualSate, text="Right", image = rightic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.x_right(),state="disabled")
        self.rightbt.grid(row=2, column=0, columnspan=1)

        leimg = Image.open('Icon/left.jpg')
        leics = leimg.resize((30, 30))
        leftic = ImageTk.PhotoImage(leics)
        self.leftbt = tk.Button(manualSate, text="Left", image = leftic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.x_left(),state="disabled")
        self.leftbt.grid(row=2, column=1, columnspan=1)

        simg = Image.open('Icon/hold.jpg')
        sics = simg.resize((30, 30))
        stopic = ImageTk.PhotoImage(sics)
        self.stopbt = tk.Button(manualSate, text="Hold", image = stopic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: self.hold(),state="disabled")
        self.stopbt.grid(row=3, column=0, columnspan=1)

        eimg = Image.open('Icon/exit.jpg')
        eics = eimg.resize((30, 30))
        exitic = ImageTk.PhotoImage(eics)
        self.exitbt = tk.Button(manualSate, text="Exit", image = exitic, width=100, bd=2, compound="left", cursor="exchange", command = lambda: root.destroy())
        self.exitbt.grid(row=3, column=1, columnspan=1)

        self.rrightbt = tk.Button(manualSate, text="Rotation right", width=10, bd=2, cursor="exchange", command = lambda: self.z_right(),state="disabled")
        self.rrightbt.grid(row=1, column=3, columnspan=1)

        self.rleftbt = tk.Button(manualSate, text="Rotation left", width=10, bd=2, cursor="exchange", command = lambda: self.z_left(),state="disabled")
        self.rleftbt.grid(row=2, column=3, columnspan=1)

        # self.robt = tk.Button(manualSate, text="Rotation", width=10, bd=2, cursor="exchange", command = lambda: self.z_rotation(),state="disabled")
        # self.robt.grid(row=3, column=3, columnspan=1)

        # self.orgbt = tk.Button(manualSate, text="Origin", width=10, bd=2, cursor="exchange", command = lambda: self.org(),state="disabled")
        # self.orgbt.grid(row=3, column=2, columnspan=1)

        self.upbt = tk.Button(manualSate, text="Up", width=10, bd=2, cursor="exchange", command = lambda: self.z_up(),state="disabled")
        self.upbt.grid(row=1, column=2, columnspan=1)

        self.downbt = tk.Button(manualSate, text="Down", width=10, bd=2, cursor="exchange", command = lambda: self.z_down(),state="disabled")
        self.downbt.grid(row=2, column=2, columnspan=1)

        self.initbt = tk.Button(manualSate, text="Reinitate", width=10, bd=2, cursor="exchange", command = lambda: self.reinit())
        self.initbt.grid(row=0, column=2, columnspan=1)

        self.var = tk.StringVar()
        label = tk.Label(root,textvariable= self.var, bg = "white" ,height = 2, width = 50, relief = "solid",cursor="exchange")
        label.grid(row=1, column=1, columnspan=5,sticky="W")
        
        positionSate = tk.LabelFrame(root, text="Show Current Position", width=200)
        positionSate.grid(row=4, column=0, columnspan=5,sticky="W")
        posscroll = tk.Scrollbar(positionSate) 
        posscroll.pack(side = "right", fill = "y") 
        self.poslist = tk.Listbox(positionSate, width=55, yscrollcommand = posscroll.set )  
        self.poslist.pack(side = "left", fill = "both")    
        posscroll.config(command = self.poslist.yview) 

        oriSate = tk.LabelFrame(root, text="Show Current Quaternion", width=200)
        oriSate.grid(row=5, column=0, columnspan=5,sticky="W")
        oriscroll = tk.Scrollbar(oriSate) 
        oriscroll.pack(side = "right", fill = "y") 
        self.orilist = tk.Listbox(oriSate, width=80, yscrollcommand = oriscroll.set )  
        self.orilist.pack(side = "left", fill = "both")    
        oriscroll.config(command = self.orilist.yview) 

        setposSate = tk.LabelFrame(root, text="Set Position", width=200)
        setposSate.grid(row=6, column=0, columnspan=5,sticky="W")
        xlabel = tk.Label(setposSate, text="x:")
        xlabel.grid(row=0, column=0, columnspan=1)
        self.xentry = tk.Entry(setposSate) 
        self.xentry.grid(row=0, column=1, columnspan=1)
        ylabel = tk.Label(setposSate, text="y:")
        ylabel.grid(row=0, column=2, columnspan=1)
        self.yentry = tk.Entry(setposSate) 
        self.yentry.grid(row=0, column=3, columnspan=1)
        zlabel = tk.Label(setposSate, text="z:")
        zlabel.grid(row=0, column=4, columnspan=1)
        self.zentry = tk.Entry(setposSate) 
        self.zentry.grid(row=0, column=5, columnspan=1)
        rzlabel = tk.Label(setposSate, text=" Rotation z:")
        rzlabel.grid(row=0, column=6, columnspan=1)
        self.rzentry = tk.Entry(setposSate) 
        self.rzentry.grid(row=0, column=7, columnspan=1)

        self.setposbt = tk.Button(setposSate, text="Set Position", width=10, bd=2, cursor="exchange", command = lambda: self.setpos())
        self.setposbt.grid(row=1, column=3, columnspan=1)
        
        self.clearbt = tk.Button(setposSate, text="Clear", width=10, bd=2, cursor="exchange", command = lambda: self.clear())
        self.clearbt.grid(row=1, column=1, columnspan=1)

        root.mainloop()
    
    def main(self): 
        self.rate = rospy.Rate(20.0)
        # Subscribe to drone state
        rospy.Subscriber('mavros/state', State, self.state_cb)
        # Subscribe to drone's local position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pos_cb)
        # Setpoint publisher  
        self.sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(20.0)
        self.modes="manual"
        self.st = "C"
        self.setArm()
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        while k<10:
            self.sp_pub.publish(self.ps)
            self.rate.sleep()
            k = k + 1
            print("init setpoint")

        # activate OFFBOARD mode
        self.setOffboardMode()

        while not rospy.is_shutdown():
            rospy.loginfo("Mode: %s State: %s",self.modes, self.st)
            if self.st == "C":
                print("init")                
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                if (self.state.armed == True):
                    self.var.set("Select a mode")
                else:
                    self.var.set("Need Reinitiate")
            elif self.modes == "manual" and self.st == "T": #Takeoff
                self.takeoff()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "D": #Landing
                self.landing()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "F": #forward
                self.y_forward()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "B": #Backward
                self.y_back()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z)) 
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "R": #right
                self.x_right()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "L": #left
                self.x_left()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.modes == "manual" and self.st == "X": #Hold
                self.hold()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "A":
                self.z_right()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "S":
                self.z_left()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "O":
                self.org()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.st == "E":
                print("Exit")
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                if (self.state.armed == False):
                    self.var.set("Landing successfully and disarmed ")
                else:
                    self.var.set("Landing successfully")
            elif self.modes == "manual" and self.st == "W":
                self.z_up()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
            elif self.modes == "manual" and self.st == "N":
                self.z_down()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)     
            elif self.st == "I":
                self.reinit()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
                if (self.state.armed == True):
                    self.var.set("Select a mode")
                else:
                    self.var.set("Need Reinitiate") 
            elif self.modes == "manual" and self.st == "Q":
                self.z_rotation()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)  
            elif self.modes == "manual" and self.st == "P":
                self.setpos()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps) 
            elif self.modes == "auto" and self.st == "U":
                self.auto()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
            elif self.modes == "manual" and self.st == "M":
                self.manual()
                self.poslist.insert(0, "local position x: {} local position y: {} local position z: {}".format(self.local_pos_x,self.local_pos_y,self.local_pos_z))
                self.orilist.insert(0, "local quaternion x: {} local quaternion y: {} local quaternion z: {} local quaternion w: {}".format(self.local_quat_x,self.local_quat_y,self.local_quat_z,self.local_quat_w))
                self.sp_pub.publish(self.ps)
                if (self.state.armed == True):
                    self.var.set("Change to Manual mode")
                else:
                    self.var.set("Need Reinitiate")

if __name__ == '__main__':
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True, disable_signals=True)
    my_node = control_node()
    t = threading.Thread(target = my_node.gui,daemon = True)
    t.start()
    my_node.main()

    
