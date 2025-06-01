# This is a Python module that allows driving a Mecademic robot.
# This Python module can be run directly in console mode to test its functionality.
# This module allows communicating with a robot through the command line.
# The same commands we can input manually are used by RoboDK to drive the robot from the PC.
# RoboDK Drivers are located in /RoboDK/api/Robot/ by default. Drivers can be PY files or EXE files.
#
# Drivers are modular. They are not part of the RoboDK executable but they must be placed in C:/RoboDK/api/robot/, then, linked in the Connection parameters menu:
#   1. right click a robot in RoboDK, then, select "Connect to robot".
#   2. In the "More options" menu it is possible to update the location and name of the driver.
# Driver linking is automatic for currently available drivers.
#
# Alternatively to the standard programming methods (where a program is generated, then, transferred to the robot and executed) it is possible to run a program simulation directly on the robot
# The robot movement in the simulator is then synchronized with the real robot.
# Programs generated from RoboDK can be run on the robot by right clicking the program, then selecting "Run on robot".
#   Example:
#   https://www.youtube.com/watch?v=pCD--kokh4s
#
# Example of an online programming project:
#   https://robodk.com/blog/online-programming/
#
# It is possible to control the movement of a robot from the RoboDK API (for example, from a Python or C# program using the RoboDK API).
# The same code is used to simulate and optionally move the real robot.
#   Example:
#   https://robodk.com/offline-programming
#
#   To establish connection from RoboDK API:
#   https://robodk.com/doc/en/PythonAPI/robolink.html#robolink.Item.ConnectSafe
#
# Example of a quick manual test in console mode:
#  User entry: CONNECT 192.168.123.1
#  Response:   SMS:Response from the robot or failure to connect
#  Response:   SMS:Ready 
#  User entry: MOVJ 10 20 30 40 50 60
#  Response:   SMS:Working...
#  Response:   SMS:Ready
#  User entry: CJNT
#  Response:   SMS:Working...
#  Response:   JNTS: 10 20 30 40 50 60
#
#---------------------------------------------------------------------------------

import sys
import time
import socket
import threading
try:
    # Python3
    import queue    
except ImportError:
    # Python2
    import Queue as queue

# lock for scan buffer
cmdlock = threading.Lock()

# MECA_R_VERSION should be 1 or 2 for older Meca robots
MECA_R_VERSION = 3 

# Time to wait for a reply. Increase this value if network requires it
TIME_WAIT_RESPONSE = 0.2 

#----------- Communication class for the Mecademic robot -------------
def msg_info(robot_msg):
    if robot_msg is None:
        return False, -1, "No communication"
        
    problems = False
    msg_id = int(robot_msg[1:5])
    msg_str = robot_msg[7:-2]

    # msg_id = 1000 to 1500 are error codes
    if msg_id < 1500:
        problems = True            
    else:
        # Other error codes
        error_codes = [3001, 3003]
        if msg_id in error_codes:
            problems = True
            
    if problems:
        print_message(msg_str)        
    return problems, msg_id, msg_str

# This class handles communication between this driver (PC) and the Meca robot
class MecaRobot:
    """Robot class for programming Mecademic robots"""
    LAST_MSG = ""       # Keep a copy of the last message received
    CONNECTED = False   # Connection status is known at all times
    
    # This is executed when the object is created
    def __init__(self):
        self.BUFFER_SIZE = 5120 # bytes
        self.TIMEOUT = 60 # seconds # End of block can take a long time if we buffer the movements
        self.PACKET_COUNT = int(0)
        self.Rounding = 0
        #self.TIMEOUT = 10 # seconds
        self.sock = None
        self.sockjnts = None
        
    def __del__(self):
        self.disconnect()
        
    # Disconnect from robot
    def disconnect(self):
        self.CONNECTED = False
        if self.sock != None:
            #self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            print_message('Connection closed')
            self.sock = None
        
        if self.sockjnts != None:
            #self.sockjnts.shutdown(socket.SHUT_RDWR)
            self.sockjnts.close()
            self.sockjnts = None

        UpdateStatus(ROBOTCOM_DISCONNECTED)
        return True
        
    def Stop(self):
        global ROBOT_MOVING
        ROBOT_MOVING = False
        return self.Run("ClearMotion", None, False, True)            
    
    # Connect to robot
    def connect(self, ip, port=10000):
        # This pause provokes required TCP flushing and it may be required for older Meca robots
        pause_com = 0.0 # 0.1 
        
        global ROBOT_MOVING
        self.disconnect()
        print_message('Connecting to robot %s:%i' % (ip, port))
        # Create new socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(4)
        self.sockjnts = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sockjnts.settimeout(4)        
        UpdateStatus(ROBOTCOM_WORKING)
        # Communication ports for Mecademic are 10000 and 10001
        self.sock.connect((ip, port))
        self.CONNECTED = True
        ROBOT_MOVING = False
        print_message('Waiting for welcome message...')
        UpdateStatus(ROBOTCOM_WORKING)
        #time.sleep(2)
        
        # receive welcome message and output to the log
        problems, msg_id, msg_str = msg_info(self.recv_str())
        if problems:
            print(msg_str)
            self.disconnect()            
            return False
        
        # notify status that the robot is still working
        UpdateStatus(ROBOTCOM_WORKING)
        
        # send activate robot and read confirmation        
        self.sock.settimeout(10)
        print_message('Reset errors...')
        import time
        time.sleep(pause_com)
        self.Run('ResetError', sync=True, send_ready=False)
        print_message('Activating robot...')
        time.sleep(pause_com)
        self.Run('ActivateRobot', sync=True, send_ready=False)
        self.sock.settimeout(30)  
        print_message('Moving home...')
        time.sleep(pause_com)
        self.Run('Home', sync=True, send_ready=False)
        self.sock.settimeout(4)
        #self.Run('ClearMotion', sync=True, send_ready=False)#stop motion
        # RoboDK provides xyzwpr data for the TCP with respect to the robot reference frame for linear movements
        
        print_message('Updating TRF...')
        time.sleep(pause_com)        
        # This may not be 100% needed
        # Important: sync must be set to false
        self.Run('SetTRF', [0, 0, 0, 0, 0, 0], sync=False, send_ready=False)
        # Important! It does not always return an answer! sync should be false        
        
        print_message('Updating WRF...')
        time.sleep(pause_com)
        # Important: sync must be set to false
        self.Run('SetWRF', [0, 0, 0, 0, 0, 0], sync=False, send_ready=False)
        
        # Synchronization required!
        #time.sleep(0.25)
        self.Run('GetStatusRobot', sync=True, send_ready=True)

        self.sock.settimeout(self.TIMEOUT)        
        self.sockjnts.connect((ip, 10001))
        
        q = queue.Queue()
        t = threading.Thread(target=robot_monitor, args=(q, self))
        t.daemon = True
        t.start()
        return True

    # Send a line to the robot through the communication port (TCP/IP)
    def send_str(self, msg):
        try:
            sent = self.sock.send(bytes(msg+'\0','ascii'))
            if sent == 0:
                return False
            return True
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print_message("Connection aborted when sending data")
            print_message(str(e))
            return False
        except Exception as e:
            print_message(str(e))
            print_message("Connection problems (1)")
            return False
    
    # Receive a line from the robot through the communication port (TCP/IP)
    def recv_str(self):
        bdata = b''
        try:
            #start_wait = time.time()
            #while (time.time()-start_wait) < self.TIMEOUT
            #    bdata = self.sock.recv(self.BUFFER_SIZE)
            #    break
            #self.PACKET_COUNT = self.PACKET_COUNT + 1
            bdata = self.sock.recv(self.BUFFER_SIZE)            
            
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print_message("Connection aborted while waiting for data")
            print_message(str(e))
            return None
        except Exception as e:
            print_message(str(e))
            print_message("Connection problems (2)")
            return None
            
        if bdata == b'':
            return None
            
        self.LAST_MSG = bdata.decode('ascii')
        return self.LAST_MSG
        
    def Synchronize(self):
        # Force synchronization
        self.Run('Delay', 0, sync=True)
        
    # Run a specific command and provide required parameters   
    def Run(self, cmd, values=None, send_ready=True, sync=None):
        if sync is None:
            sync = self.Rounding <= 0
            
        # Skip the command if the robot is not connected
        if not self.CONNECTED:
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return
        
        if isinstance(values, list):
            str_send = cmd + '(' + (','.join(format(vi, ".6f") for vi in values)) + ')'
        elif values is None:
            str_send = cmd
        else:
            str_send = cmd + '(' + str(values) + ')'

        #---------- Send robot command -------
        # Notify RoboDK (debug only)
        # print('Sending: %s' % str_send)
        UpdateStatus(ROBOTCOM_WORKING)

        # Try to send the command
        if self.send_str(str_send) is False:
            print_message("Robot connection broken")
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return False
        
        #if self.Rounding > 0 and cmd.startswith("Move"):
        #    UpdateStatus(ROBOTCOM_READY)
        #    return True
        
        if not sync:
            if send_ready:
                UpdateStatus(ROBOTCOM_READY)
            return True
        
        # It is important to wait for the response when it is needed
        time.sleep(TIME_WAIT_RESPONSE)
            
        # Try to receive a response
        robot_msg = self.recv_str()
        if self.LAST_MSG is None:
            print_message("Robot connection broken")
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return False

        problems, msg_id, msg_str = msg_info(robot_msg)
        if problems:
            return False
        
        # Support for old and new versions
        if msg_id == 2026 or msg_id == 2102:
            # robot response after a GetJoints request
            # [2026][j1, j2, j3, j4, j5, j6]
            print_joints(msg_str.replace(',',' '))
            UpdateStatus(ROBOTCOM_READY)
            return True
        
        # Any other acknowledge message (assumed to be successful)
        # By default, we will send the command Ready at every instruction (one Run per instruction in general)
        # print(robot_msg)
        if send_ready:
            UpdateStatus(ROBOTCOM_READY)
        else:
            # Save the Ready status to send later and notify RoboDK that the instruction was completed
            global STATUS
            STATUS = ROBOTCOM_READY
            
        return True

# Receives a string through TCP/IP. It reads until if finds NULL character
def read_line(socket):    
    string = b''
    chari = socket.recv(1)
    while chari != b'\0': # read until NULL
        string = string + chari
        chari = socket.recv(1)
    return str(string.decode('ascii')) # python 2 and python 3 compatible
        
# Specific thread to monitor robot communication
# This thread establishes a permanent link between the robot and the PC to retrieve the robot position at all times
# The robot position is displayed only when the robot is executing a motion command
# When the communication link is broken it will notify the user
def robot_monitor(q, robot):
    try:
        while robot.sockjnts:
            bdata = b''
            robot_msg = read_line(robot.sockjnts)
            #bdata = robot.sockjnts.recv(512)
            #if bdata == b'':
            #    print_message("Invalid monitoring response")
            #    return                
            #robot_msg = bdata.decode('ascii')
            msg_id = int(robot_msg[1:5])
            # Support for old and new versions:
            if msg_id == 3007 or msg_id == 2102:
                # monitoring stream of data: [3007][j1, j2, j3, j4, j5, j6]
                print_joints(robot_msg[7:-2].replace(',',' '), True)
            elif msg_id == 3010 or msg_id == 2103:
                # position data is also part of the stream. Ignore
                # [3010][212.985, -93.965, 34.273, 180.000, -1.967, 63.936]
                pass
            elif msg_id == 3000:
                # Welcome message is:
                # [3000][Connected to Mecademic Meca500 Robot.]
                pass
            else:
                print_message(robot_msg)
                print_message("Unknown monitoring response")
                return
                
    except Exception as e:
            print_message(str(e))
            print_message("Robot not connected (monitoring stopped)")
            UpdateStatus(ROBOTCOM_DISCONNECTED)
            return
        
        
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
# Generic RoboDK driver for a specific Robot class
global ROBOT
global ROBOT_IP
global ROBOT_PORT
global ROBOT_MOVING
global ROBOT_MONITOR_LAST_JOINTS

ROBOT = MecaRobot()
ROBOT_IP = "10.10.0.5"      # IP of the robot
ROBOT_PORT = 10000          # Communication port of the robot
ROBOT_MOVING = False
ROBOT_MONITOR_LAST_JOINTS = None


#------------ robot connection -----------------
# Establish connection with the robot
def RobotConnect():
    # Important: try to first disconnect:
    RobotDisconnect()
    
    global ROBOT
    global ROBOT_IP
    
    # Try to connect
    try:
        ROBOT.connect(ROBOT_IP)
    except ConnectionRefusedError:
        #print_message("Connection refused. You may need to reboot your Meca robot.")
        UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
        print_message("Connection Refused: Reboot Meca robot.")
    except ConnectionAbortedError:
        print_message("Connection aborted")
        UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
    
    
# Disconnect from the robot
def RobotDisconnect():
    global ROBOT
    global ROBOT_MOVING
    ROBOT_MOVING = False
    ROBOT.disconnect()
    
        
#-----------------------------------------------------------------------------
# Generic RoboDK driver tools

# Note, a simple print() will flush information to the log window of the robot connection in RoboDK
# Sending a print() might not flush the standard output unless the buffer reaches a certain size

def print_message(message):
    """print_message will display a message in the log window (and the connexion status bar)"""
    print("SMS:" + message)
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

def show_message(message):
    """show_message will display a message in the status bar of the main window"""
    print("SMS2:" + message)
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

def print_joints(joints, ismoving = False):
    global ROBOT_MONITOR_LAST_JOINTS
    if ismoving:
        ROBOT_MONITOR_LAST_JOINTS = joints
        # Display the feedback of the joints when the robot is moving
        if ROBOT_MOVING:
            #print("CJNT_MOVING " + " ".join(format(x, ".5f") for x in joints)) # if joints is a list of float
            print("JNTS_MOVING " + joints)
    else:
        #print("CJNT " + " ".join(format(x, ".5f") for x in joints)) # if joints is a list of float
        print("JNTS " + joints)
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

# ---------------------------------------------------------------------------------
# Constant values to display status using UpdateStatus()
ROBOTCOM_UNKNOWN                = -1000
ROBOTCOM_CONNECTION_PROBLEMS    = -3
ROBOTCOM_DISCONNECTED           = -2
ROBOTCOM_NOT_CONNECTED          = -1
ROBOTCOM_READY                  =  0
ROBOTCOM_WORKING                =  1
ROBOTCOM_WAITING                =  2

# Last robot status is saved
global STATUS
STATUS = ROBOTCOM_DISCONNECTED

# UpdateStatus will send an appropriate message to RoboDK which will result in a specific coloring
# for example, Ready will be displayed in green, Waiting... will be displayed in Yellow and other messages will be displayed in red
def UpdateStatus(set_status=None):
    global STATUS
    if set_status is not None:
        STATUS = set_status
        
    if STATUS == ROBOTCOM_CONNECTION_PROBLEMS:
        print_message("Connection problems")
    elif STATUS == ROBOTCOM_DISCONNECTED:
        print_message("Disconnected")
    elif STATUS == ROBOTCOM_NOT_CONNECTED:
        print_message("Not connected")
    elif STATUS == ROBOTCOM_READY:
        print_message("Ready")
    elif STATUS == ROBOTCOM_WORKING:
        print_message("Working...")
    elif STATUS == ROBOTCOM_WAITING:
        print_message("Waiting...")
    else:
        print_message("Unknown status");

# Sample set of commands that can be provided by RoboDK of through the command line
def TestDriver():    
    RunCommand("CONNECT 192.168.0.100 10000")
    #RunCommand("CJNT")
    
    #RunCommand("SETTOOL -0.025 -41.046 50.920 60.000 -0.000 90.000")
    #RunCommand("MOVJ -5.362010 46.323420 20.746290 74.878840 -50.101680 61.958500")
    #RunCommand("SPEED 250")
    #RunCommand("MOVL 0 0 0 0 0 0 -5.362010 50.323420 20.746290 74.878840 -50.101680 61.958500")
    #RunCommand("PAUSE 2000") # Pause 2 seconds

#-------------------------- Main driver loop -----------------------------
# Read STDIN and process each command (infinite loop)
# IMPORTANT: This must be run from RoboDK so that RoboDK can properly feed commands through STDIN
# This driver can also be run in console mode providing the commands through the console input

LISTCMD = []

def RunConsole():
    global LISTCMD
    global STATUS
    for linecmd in sys.stdin:
        linecmd = linecmd.strip()
        if linecmd.startswith("DISCONNECT"):
            UpdateStatus(ROBOTCOM_WORKING)
            # clear the list and disconnect from robot
            LISTCMD = []            
            RobotDisconnect()
            UpdateStatus(ROBOTCOM_DISCONNECTED)
        elif linecmd.startswith("STOP") or linecmd.startswith("s"):
            # clear the list and disconnect from robot
            if STATUS != ROBOTCOM_READY:
                # We can't stop the robot if it is ready
                UpdateStatus(ROBOTCOM_WORKING)
                LISTCMD = []
                def robot_stop():
                    global ROBOT
                    ROBOT.Stop()
                    
                robot_stop()
            # Will send ready on success
            #threading.Thread(target=robot_stop).start()            
            #import time
            #time.sleep(0.1)
            #robot_stop()
            #RobotConnect()
            
            # Reconnect again
            LISTCMD = []            
            
        elif linecmd.startswith("QUIT"):
            # Stop the driver
            UpdateStatus(ROBOTCOM_WORKING)
            RobotDisconnect()
            UpdateStatus(ROBOTCOM_DISCONNECTED)
            quit(0) # Stop the driver
        else:
            LISTCMD.append(linecmd)

def RunDriver():
    while True:
        if len(LISTCMD) > 0:
            RunCommand(LISTCMD.pop(0))
            
def RunDriverThread():
    t = threading.Thread(target=RunDriver)
    t.daemon = True
    t.start()
                
# Each line provided through command line or STDIN will be processed by RunCommand    
def RunCommand(linecmd):
    global ROBOT_IP
    global ROBOT
    global ROBOT_MOVING
    
    # strip a line of words into a list of numbers
    def line_2_values(words):
        values = []        
        for word in words:
            try:
                number = float(word)
                values.append(number)
            except:
                pass
        return values
    
    linecmd = linecmd
    words = linecmd.split(' ')
    values = line_2_values(words)
    nvalues = len(values)
    nwords = len(words)
    
    if linecmd == "":
        # Skip if no command is provided
        return
    
    elif nwords >= 2 and linecmd.startswith("CONNECT"):
        # Connect to robot provided the IP and the port
        ROBOT_IP = words[1]
        if nwords >= 3 and nvalues >= 1 and values[0] != 10000:
            #ROBOT_PORT = values[0]
            print("Using default port 10000, not %i" % ROBOT_PORT)
            
        RobotConnect()
    
    elif nvalues >= 6 and linecmd.startswith("MOVJ"):
        # Activate the monitor feedback
        ROBOT_MOVING = True
        
        # Execute a joint move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        ROBOT.Run('MoveJoints', values[0:6])

    elif nvalues >= 12 and linecmd.startswith("MOVL"):
        # Activate the monitor feedback
        ROBOT_MOVING = True
        
        # Execute a linear move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        ROBOT.Run('MoveLin', values[6:12])
        
    elif linecmd.startswith("CJNT"):
        # Retrieve the current position of the robot
        if ROBOT_MONITOR_LAST_JOINTS != None:
            print_joints(ROBOT_MONITOR_LAST_JOINTS)
            UpdateStatus(ROBOTCOM_READY)
        else:
            ROBOT.Run('GetJoints', sync=True)    

    elif nvalues >= 1 and linecmd.startswith("SPEED"):
        # First value is linear speed in mm/s
        # IMPORTANT! We should only send one "Ready" per instruction
        if values[0] > 0:            
            # make sure we do not exceed maximum speed (robot turns into error mode)            
            if MECA_R_VERSION > 2:
                ROBOT.Run('SetCartLinVel', min(500.0, values[0]), False)
            else:
                ROBOT.Run('SetCartVel', min(500.0, values[0]), False)
            
        
        # Second value, if available, is joint speed in deg/s
        if nvalues >= 2 and values[1] > 0:
            # make sure we do not exceed maximum speed (robot turns into error mode)
            ROBOT.Run('SetJointVel', min(135.0, values[1]), False)
        
        # Third value, if available, is linear acceleration in mm/s2
        if nvalues >= 3 and values[2] > 0:
            print("Warning: Setting linear acceleration not supported")
        
        # Fourth value, if available, is joint acceleration in deg/s2
        if nvalues >= 4 and values[3] > 0:
            print("Warning: Setting joint acceleration not supported")        

        # Provokes sending Ready:
        UpdateStatus()
    elif nvalues >= 1 and linecmd.startswith("SETROUNDING"):
        # Set the rounding/smoothing value. Also known as ZoneData in ABB or CNT for Fanuc
        ROBOT.Rounding = values[0]
        if MECA_R_VERSION > 2:
            # SetCornering changed to setblending for R3 robots
            ROBOT.Run('SetBlending', [min(values[0],100)] if values[0] > 0 else [0])
        else:
            ROBOT.Run('SetCornering', [1] if values[0] > 0 else [0])       
    
    elif nvalues >= 1 and linecmd.startswith("PAUSE"):
        UpdateStatus(ROBOTCOM_WAITING)
        
        # Force synchronization (end of block)
        ROBOT.Synchronize()
        
        # Run a pause
        if values[0] > 0:
            import time
            time.sleep(values[0] * 0.001)
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 2 and linecmd.startswith("SETDO"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = values[0]
        dIO_value = values[1]
        print_message("Warning: Setting DO[%i] = %.1f not implemented" % (dIO_id, dIO_value))
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 2 and linecmd.startswith("WAITDI"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = values[0]
        dIO_value = values[1]
        print_message("Warning: Waiting DI[%i] = %.1f not implemented" % (dIO_id, dIO_value))
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 6 and linecmd.startswith("SETTOOL"):
        # Set the Tool reference frame provided the 6 XYZWPR values by RoboDK
        ROBOT.Run('SetTRF', values)

    elif nvalues >= 1 and nwords >= 2 and linecmd.startswith("RUNPROG"):
        UpdateStatus(ROBOTCOM_WORKING)        
        if nwords < 3:
            prog_id = int(values[0])
            prog_name = "Program %i" % prog_id            
        else:
            prog_name = words[2]
        
        run_cmd = words[2]
        if run_cmd.startswith('Gripper'):
            ROBOT.Run(run_cmd)
        else:
            print_message("Unknown program call: " + run_cmd)
            print_message("Warning: Running program %s not implemented" % (prog_name))                   
        
        UpdateStatus(ROBOTCOM_READY)
        
    elif nwords >= 2 and linecmd.startswith("POPUP "):
        UpdateStatus(ROBOTCOM_WORKING)
        message = linecmd[6:]            
        print_message("Warning: Display message %s not implemented" % (message))
        UpdateStatus(ROBOTCOM_READY)
        
    #elif linecmd.startswith("DISCONNECT"):
    #    # Disconnect from robot
    #    ROBOT.disconnect()
    #    UpdateStatus(ROBOTCOM_DISCONNECTED)
        
    elif linecmd == "t":
        # Call custom procedure for quick testing
        TestDriver()
        
    elif linecmd == "d":
        # Call custom procedure for quick testing
        RobotDisconnect()
                
    elif linecmd == "m":
        # Call custom procedure for quick testing
        global LISTCMD
        LISTCMD += ['MOVJ -10 0 0 0 0 0']
        LISTCMD += ['MOVJ 10 0 0 0 0 0']       
            
    else:
        print("Unknown command: " + linecmd)
    
    # Stop monitoring feedback
    ROBOT_MOVING = False

if __name__ == "__main__":
    """Call Main procedure"""
    
    # It is important to disconnect the robot if we force to stop the process
    import atexit
    atexit.register(RobotDisconnect)
    
    # Flush Disconnected message
    UpdateStatus()
    
    # Start parallel thread
    RunDriverThread()
    
    # Run the driver process
    RunConsole()
    
    # Test the driver with a sample set of commands
    #TestDriver()

