from sqlite3 import Time
import xpc
import time
import serial
import serial.tools.list_ports
import math 

AIL_CONST = 6
ELE_CONST = 1

E_BOUNDS = [-1, -3]
A_BOUNDS = [-1.356, 3]

MAX_FORCE = 12
MIN_FORCE = 2.5

def _map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class ConnectionState:
    CONNECTED = 1
    DISCONNECTED = 2

class Connection:
    state: ConnectionState = ConnectionState.DISCONNECTED
    connection_obj = None

    def __init__(self):
        pass

    def _update_state(self):
        pass

    def _connect(self):
        pass

    def connection(self):
        self._update_state()
        if self.state == ConnectionState.DISCONNECTED:
            self._connect()
    
        return self.connection_obj
    

class XplaneConnection(Connection):
    def __init__(self):
        super().__init__()
        self.connection_obj = xpc.XPlaneConnect()

    def _update_state(self):
        try:
            # If X-Plane does not respond to the request, a timeout error
            # will be raised.
            self.connection_obj.getDREF("sim/test/test_float")
            if self.state == ConnectionState.DISCONNECTED:
                print("Established Connection to X-Plane.")

            self.state = ConnectionState.CONNECTED

        except:
            if self.state == ConnectionState.CONNECTED:
                print("Lost Connection to X-Plane.")
            self.state = ConnectionState.DISCONNECTED
        
    def _connect(self):
        self._update_state()

    

class SerialConnection(Connection):
    def __init__(self):
        super().__init__()

    def _update_state(self):
        if self.connection_obj is None:
            self._connect()
        else:
            if self.connection_obj.is_open:
                self.state = ConnectionState.CONNECTED
            elif self.state == ConnectionState.CONNECTED:
                print("Lost Connection to Arduino.")
                self.connection_obj = None
                self.state = ConnectionState.DISCONNECTED


    def _connect(self):
        ports = list(serial.tools.list_ports.comports())
        try:
            for p in ports:
                if 'CP2102' in p.description:
                    self.connection_obj = serial.Serial(p.device, 115200)
                    print("Established Connection to Arduino. Waiting for motor to start.")
                    line = ""
                    while line != "READY":
                        try:
                            line = self.connection_obj.readline().decode('utf-8').strip()
                        
                        except UnicodeDecodeError:
                            pass

                    self.state = ConnectionState.CONNECTED

            if self.state == ConnectionState.DISCONNECTED:
                print("No Arduino Found.")

        # except if the port is busy
        except serial.SerialException:
            print("Arduino is busy.")
            self.state = ConnectionState.DISCONNECTED



def ffb_control(xp, serial):

    while True:
        #get indicated airspeed
        airspeed = xp.getDREF("sim/flightmodel/position/indicated_airspeed")[0]

        # read line from serial and convert to list separated by commas
        try:
            serial.flushInput()
            line = serial.readline().decode('utf-8').strip().split('\t')
            if len(line) == 4: # motor monitor line
                pitch_cmd = _map(float(line[3]), E_BOUNDS[0], E_BOUNDS[1], -1, 1)

            else:
                continue
        except UnicodeDecodeError:
            continue

        xp.sendCTRL([pitch_cmd, -998])

        # print(b"A" + str(round(airspeed, 2)).encode() + b"\n")

        serial.write(b"A" + str(round(airspeed, 2)).encode() + b"\n")

        # print(str(round(pitch_force, 2)))

rate_ct = 0
last_time = time.time()
def rate():
    global rate_ct
    global last_time
    rate_ct += 1
    if time.time() - last_time > 1:
        print(rate_ct)
        rate_ct = 0
        last_time = time.time()
    
xpClient = XplaneConnection()
serialClient = SerialConnection()

while True:
    try:
        serConn = serialClient.connection()
        xpConn = xpClient.connection()

        if serConn is not None and xpConn is not None:
            ffb_control(xpConn, serConn)
        else:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        break
    except:
        serialClient._update_state()
        xpClient._update_state()
