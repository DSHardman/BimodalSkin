import waypoints as wp
import kg_robot as kgr
import time
import serial
import numpy as np
import random
import nidaqmx as ni
from nidaqmx.constants import TerminalConfiguration

# Repeated xy positions & depth
# small: a: [0.0206, 0.00625], b: [0.025, 0.01], c: [0, 0.0025]
# medium: a: [0.03, 0.03], b: [0.0375, 0.021], c: [0, 0.0033]
# large: a: [0.041, 0.039], b: [0.051, 0.02849], c: [0, 0.0044]

xa = 0.041
ya = 0.039

xb = 0.051
yb = 0.02849

xc = 0
yc = 0.0044


def repeats(x, y, depth, rootstring, timepressed=10, timeafter=5):
    com = "COM3"  # Of probe

    # LINES 39 & 40 CHANGE WHICH POINT IS BEING PROBED

    timebefore = 1
    timedown = 1.5
    timeup = 1.5
    dt = 0.05

    duration = timebefore + timeafter + timedown + timeup + timepressed
    samplesdown = int(timedown/dt)
    samplesup = int(timeup/dt)

    zeropose = [0.179466, -0.55881, -0.00290189, -3.11991, -0.121519, -0.0376175]

    #  Connect to UR5
    urnie = kgr.kg_robot(port=30010, db_host="169.254.150.50")
    urnie.set_tcp(wp.probing_tcp)

    print(urnie.getl())

    # Connect to probe COM port
    if 'ser' in globals() and not ser.isOpen():
        ser = serial.Serial(port=com, baudrate=9600)
    elif 'ser' in globals() and ser.isOpen():
        ser.flushInput()
        ser.flushOutput()
    else:
        ser = serial.Serial(port=com, baudrate=9600)

    #  Set positive rail to 5V
    with ni.Task() as task:
        task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        task.write(5)

    for i in range(5):  # Same location probed 5 times
        xy = [x, y, depth]

        # Control press using defined variables
        startingpose = np.add(zeropose, [x, y, 0.01, 0, 0, 0])
        urnie.movel(startingpose, acc=0.02, vel=0.02)

        poses = 0
        poses = np.ones((int(duration/dt), 1))*startingpose

        for j in range(int(timebefore/dt), int(timebefore/dt) + samplesdown):
            poses[j] = np.add(poses[j], [0, 0, -(depth+0.01)*(j - int(timebefore/dt))/samplesdown, 0, 0, 0])

        for j in range(int(timebefore/dt) + samplesdown, int((timebefore+timedown+timepressed)/dt)):
            poses[j] = np.add(poses[j], [0, 0, -(depth+0.01), 0, 0, 0])

        for j in range(int((timebefore+timedown+timepressed)/dt), int((timebefore+timedown+timepressed)/dt) + samplesup):
            poses[j] = np.add(poses[j], [0, 0, -(depth+0.01) + ((depth+0.01)*(j - int((timebefore+timedown+timepressed)/dt))/samplesup),
                                         0, 0, 0])

        urnie.movel(poses[0], acc=0.02, vel=0.02)

        # Measure and record sensor data
        with ni.Task() as task:
            task.ai_channels.add_ai_voltage_chan("Dev1/ai0:15", terminal_config=TerminalConfiguration.RSE)

            urnie.movel(poses[0], acc=0.02, vel=0.02)
            data = np.zeros((int(duration/dt), 1))*[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            times = np.zeros((int(duration/dt), 1))
            t0 = time.time()
            temp = ser.readline()
            for k in range(0, int(duration/dt)):
                urnie.servoj(poses[k], control_time=dt, lookahead_time=0.2)
                data[k] = task.read()
                times[k] = time.time() - t0
                while time.time() - t0 < dt*(k+1):
                    continue

        urnie.movel(startingpose, acc=0.02, vel=0.02)

        # Save data
        np.save('rep/rawdata/response'+rootstring+str(i), data)
        np.save('rep/rawdata/poses'+rootstring+str(i), poses)
        np.save('rep/rawdata/times'+rootstring+str(i), times)
        np.save('rep/rawdata/xy'+rootstring+str(i), xy)
        np.save('rep/rawdata/temp'+rootstring+str(i), float(temp))

        print(i)

    urnie.close()


repeats(xa, ya, 0.001, '_a1le_100_')
repeats(xa, ya, 0.004, '_a4le_100_')
repeats(xb, yb, 0.001, '_b1le_100_')
repeats(xb, yb, 0.004, '_b4le_100_')
repeats(xc, yc, 0.001, '_c1le_100_')
repeats(xc, yc, 0.004, '_c4le_100_')

repeats(xa, ya, 0.004, '_a4le_100_q_', 1, 1)
repeats(xa, ya, 0.004, '_a4le_100_s_', 100, 50)
repeats(xa, ya, 0.001, '_a1le_100_q_', 1, 1)
repeats(xb, yb, 0.004, '_b4le_100_s_', 100, 50)
