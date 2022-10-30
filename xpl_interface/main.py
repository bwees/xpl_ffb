from sqlite3 import Time
import xpc
import time
import serial
import serial.tools.list_ports

AIL_CONST = 6
ELE_CONST = 6


class Connection:
    def __init__(self):
        pass

    def connect(self):
        pass

    def get_connection(self):
        pass

def connect_xp():
    global xpConnectionState
    global xp_conn

    client = xpc.XPlaneConnect()
    # Verify connection
    try:
        # If X-Plane does not respond to the request, a timeout error
        # will be raised.
        client.getDREF("sim/test/test_float")
        print("Established Connection to X-Plane.")
        xpConnectionState = ConnectionState.Connected
        return client
    except:
        print("BIG ERROR")
        raise ConnectionError("Connection to X-Plane failed.")

def connect_serial():
    global serialConnectionState
    global ser_conn

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'CP2102' in p.description:
            ser = serial.Serial(p.device, 115200)
            print("Established Connection to Arduino.")
            serialConnectionState = ConnectionState.Connected
            return ser


def ffb_control():
    client = connect_xp()
    ser = connect_serial()

    while True:
        sim_paused = client.getDREF("sim/time/paused")[0] == 1

        #get indicated airspeed
        airspeed = client.getDREF("sim/flightmodel/position/indicated_airspeed")[0]

        relative_force = airspeed**2


        # get joystick inputs
        joy = client.getCTRL()
        pitch_cmd = joy[0]
        roll_cmd = joy[1]

        pitch_force = pitch_cmd * relative_force
        roll_force = roll_cmd * relative_force

        # cap the pitch force at 12v
        if pitch_force > 12:
            pitch_force = 12
        elif pitch_force < -12:
            pitch_force = -12


        ser.write(b"M" + str(round(pitch_force * ELE_CONST)).encode() + b"\n")
        print(str(round(pitch_force * ELE_CONST, 2)))




    

class ConnectionState:
    Connected = 1
    Awaiting = 2
    Dropped = 3


while True:
    try: 
        ffb_control()
    except ConnectionError as e:
        print(e)
        if xpConnectionState != ConnectionState.Awaiting:
            print("Waiting for X-Plane Connection")
            xpConnectionState = ConnectionState.Awaiting
    except TimeoutError:
        if xpConnectionState != ConnectionState.Dropped:
            print("Connection to X-Plane was lost")
            xpConnectionState = ConnectionState.Dropped

    time.sleep(1)