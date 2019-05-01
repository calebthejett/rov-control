# This requires the xboxdrv package through system package manager
# and xbox.py (wget https://raw.githubusercontent.com/FRC4564/Xbox/master/xbox.py)
# and pyserial available from pip
# and simple-pid available from pip

#controller description
# LB - trim down
# RB - trim up
# left y - forward/back
# left x - turn left/right
# right y - up/down
# right x - strafe left/right
# back - estop

#transmit packet description
# first character is type of message-
# # number 0-7           ESC value
# # i                    get pitch and roll from bot
# # a                    request for 12v bus current
# # A                    request for 12VA bus current


import xbox # driver for xbox controller
import serial # for tether communication
import time
import sys
import math
from simple_pid import PID #pid handling for leveling

# set default values
power_scale = 0.7

pid_pitch = PID(0.3,0.05,0.001,setpoint=0)
pid_roll = PID(0.3,0.05,0.001,setpoint=0)
pid_pitch.sample_time = 0.05
pid_roll.sample_time = 0.05
up_offset = 0
translation_factor = 0.2
rotation_factor = 0.2
up_button_state = 0
down_button_state = 0
start_button_state = 0
motors_en = False

const_deadzone = 0.01
last_time = 0
last_print = 0

output_stream = sys.stdout

pitch_ctrl = 0
roll_ctrl = 0
last_pid = 0
pitch = 0
roll = 0
pitch_offset = 0
roll_offset = 0
main_xbox_ctrlr = None

def init():
    global main_xbox_ctrlr
    global pid_pitch
    global pid_roll
    global up_offset
    global translation_factor
    global rotation_factor
    global up_button_state
    global start_button_state
    global down_button_state
    global motors_en
    global const_deadzone
    global last_time
    global last_print
    global output_stream
    global pitch_ctrl
    global roll_ctrl
    global last_pid
    global pitch
    global roll
    global pitch_offset
    global roll_offset

    try:
        main_xbox_ctrlr.close()
    except:
        print("controller cannot close")

    main_xbox_ctrlr = xbox.Joystick() #setup the xbox controller

    pid_pitch = PID(0.3,0.05,0.001,setpoint=0)
    pid_roll = PID(0.3,0.05,0.001,setpoint=0)
    pid_pitch.sample_time = 0.05
    pid_roll.sample_time = 0.05
    up_offset = 0.15
    translation_factor = 0.2
    rotation_factor = 0.2
    up_button_state = 0
    down_button_state = 0
    start_button_state = 0
    motors_en = False

    const_deadzone = 0.01
    last_time = 0
    last_print = 0

    output_stream = sys.stdout

    pitch_ctrl = 0
    roll_ctrl = 0
    last_pid = 0
    pitch = 0
    roll = 0
    pitch_offset = 0
    roll_offset = 0

def main():
    # make sure controller is ready go
    # try:
    #     main_xbox_ctrlr.close()
    #     main_xbox_ctrlr = xbox.Joystick()
    # except:
    #     print("angry snake noises")
    global main_xbox_ctrlr
    global pid_pitch
    global pid_roll
    global up_offset
    global translation_factor
    global rotation_factor
    global up_button_state
    global start_button_state
    global down_button_state
    global motors_en
    global const_deadzone
    global last_time
    global last_print
    global output_stream
    global pitch_ctrl
    global roll_ctrl
    global last_pid
    global pitch
    global roll
    global pitch_offset
    global roll_offset

    with serial.Serial('/dev/ttyUSB0',115200, timeout = 0.2, write_timeout = 0.2) as serial_port:
        print("Waiting for serial port")
        while  serial_port.readline() == "":
            print("...")
            time.sleep(1)
        print("Starting main loop")
        while serial_port.is_open:
            try:
                if(time.time()-last_pid > 0.05):
                    last_pid = time.time()
                    serial_port.write("i")
                    serial_port.write("\n")
                    resp = serial_port.readline()
                    if not (resp[:-2] == "NODATA" or resp[:-2] == "NOIMU" or resp[:-2] == "WUT" or resp[:-2] == ""):
                        ax = float(resp)
                        ay = float(serial_port.readline())
                        az = float(serial_port.readline())
                        pitch = math.atan2(-ax,math.sqrt(ay*ay+az*az))*120.0/3.14159
                        roll = math.atan2(ay,az)*120.0/3.14159
                        pitch_ctrl = pid_pitch(pitch+pitch_offset)*0.03
                        roll_ctrl = pid_roll(roll+roll_offset)*0.03
                    else:
                        if not resp[:-2] == "":
                            print(resp[:-2])
                if(main_xbox_ctrlr.Back()):
                    pitch_offset = -pitch
                    roll_offset = -roll
                #translation factor
                if(main_xbox_ctrlr.dpadUp() and not up_button_state):
                    translation_factor = min(1,translation_factor+0.1)
                up_button_state = main_xbox_ctrlr.dpadUp()

                if(main_xbox_ctrlr.dpadDown() and not down_button_state):
                    translation_factor = max(0,translation_factor-0.1)
                down_button_state = main_xbox_ctrlr.dpadDown()

                #rotation factor
                if(main_xbox_ctrlr.dpadRight() and not right_button_state):
                    rotation_factor = min(1,rotation_factor+0.1)
                right_button_state = main_xbox_ctrlr.dpadRight()

                if(main_xbox_ctrlr.dpadLeft() and not left_button_state):
                    rotation_factor = max(0,rotation_factor-0.1)
                left_button_state = main_xbox_ctrlr.dpadLeft()
                
                #vertical trimming
                if(main_xbox_ctrlr.leftBumper() and not left_bumper_state):
                    up_offset = max(-1,up_offset-0.01)
                left_bumper_state = main_xbox_ctrlr.leftBumper()
                if(main_xbox_ctrlr.rightBumper() and not right_bumper_state):
                    up_offset = min(1,up_offset+0.01)
                right_bumper_state = main_xbox_ctrlr.rightBumper()
                
                forward = round(main_xbox_ctrlr.leftY()*translation_factor,3)
                right = round(-main_xbox_ctrlr.leftX()*translation_factor,3)
                turn_right = round(main_xbox_ctrlr.rightX()*rotation_factor,3)
                up = round(main_xbox_ctrlr.rightY()*translation_factor + up_offset,3)

                if(main_xbox_ctrlr.Start() and not start_button_state):
                    motors_en = not motors_en
                start_button_state = main_xbox_ctrlr.Start()
                if(motors_en):
                    fr = power_scale*max(-1,min(1,forward + right - turn_right))
                    fl = power_scale*max(-1,min(1,forward - right + turn_right))
                    br = power_scale*max(-1,min(1,forward + right + turn_right))
                    bl = power_scale*max(-1,min(1,forward - right - turn_right))
                    tl = power_scale*max(-1,min(1,up-pitch_ctrl-roll_ctrl))
                    tr = power_scale*max(-1,min(1,up-pitch_ctrl+roll_ctrl))
                    tb = power_scale*max(-1,min(1,up+pitch_ctrl))
                else:
                    fr=fl=br=bl=tl=tr=tb=0
                if abs(fr)<const_deadzone:
                    fr = 0
                if abs(fr)<const_deadzone:
                    fl = 0
                if abs(fr)<const_deadzone:
                    br = 0
                if abs(fr)<const_deadzone:
                    bl = 0

                serial_port.write("0")
                serial_port.write(str(fl))
                serial_port.write("\n")

                serial_port.write("1")
                serial_port.write(str(-fr))
                serial_port.write("\n")

                serial_port.write("2")
                serial_port.write(str(bl))
                serial_port.write("\n")

                serial_port.write("3")
                serial_port.write(str(-br))
                serial_port.write("\n")

                serial_port.write("4")
                serial_port.write(str(tl))
                serial_port.write("\n")

                serial_port.write("5")
                serial_port.write(str(tr))
                serial_port.write("\n")

                serial_port.write("6")
                serial_port.write(str(tb))
                serial_port.write("\n")
                if(time.time()-last_print > 0.2):
                    last_print = time.time()
                    output_stream.write("                                                                                                                                                                            \r")
                    output_stream.flush()
                    output_stream.write("f: "+str(round(forward,1))+
                                        "\tr: "+str(round(right,1))+
                                        "\tt: "+str(round(turn_right,1))+
                                        "\tu: "+str(round(up,1))+
                                        "\t\ttrans: "+str(round(translation_factor,1))+
                                        "\trot: "+str(round(rotation_factor,1))+
                                        "\tv_tr: "+str(round(up_offset,2))+
                                        "\tpitch: "+str(round(pitch+pitch_offset,2))+
                                        "\troll: "+str(round(roll+roll_offset,2))+
                                        "\tpitch_c: "+str(round(pitch_ctrl,2))+
                                        "\troll_c: "+str(round(roll_ctrl,2))+
                                        "\t\tt: "+str(round(time.time()-last_time,5))+
                                        "\r")
                    output_stream.flush()
                    last_time = time.time()
                #time.sleep(0.01)
            except serial.serialutil.SerialTimeoutException:
                print("")
                print("Timeout event occurred!")
            except ValueError:
                print("")
                print("")
                print("*angry roblox noises*")
                print("'"+ValueError.str()+"'")
                print("...resetting...")
                main_xbox_ctrlr.close()
                main_xbox_ctrlr = xbox.Joystick()
                continue
            

if __name__ == '__main__':
    try:
        init()
        main()
    except Exception as e:
        print("\t\ttotal failure\t\t")
        print 'error: {}'.format(e)
        try:
            main_xbox_ctrlr.close()
        except:
            print 'controller cannot close'
        print("...resetting...")
        main()
