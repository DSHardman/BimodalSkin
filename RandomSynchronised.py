import waypoints as wp
import kg_robot as kgr
import time
import serial
import numpy as np
import random
import nidaqmx as ni
from nidaqmx.constants import TerminalConfiguration

com = "COM3"  # Of probe

timebefore = 1
timedown = 1.5
timepressed = 10
timeup = 1.5
timeafter = 5
depthlower = 0.001
depthupper = 0.004
dt = 0.05

xupperbound = 45 * 0.001  # Only probes a square of this side length
yupperbound = 40 * 0.001

duration = timebefore + timeafter + timedown + timeup + timepressed
samplesdown = int(timedown/dt)
samplesup = int(timeup/dt)

#zeropose = [0.181232, -0.553783, -0.006, 3.0947, 0.420936, -0.0594446]  # small
#zeropose = [0.191144, -0.538177, -0.00425192, 3.12038, 0.12705, -0.0570452]  # medium
# zeropose = [0.175174, -0.557184, -0.00322906, -3.12655, -0.121154, 0.0856744]  # large

#  Connect to UR5
urnie = kgr.kg_robot(port=30010, db_host="169.254.150.50")
urnie.set_tcp(wp.probing_tcp)

# # Connect to probe COM port
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

for i in range(5000):  # Record 5000 probes

    # Random xy positions & depth
    x = random.random()*xupperbound
    y = random.random()*yupperbound
    depth = random.random()*(depthupper - depthlower) + depthlower
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
        while (1):
            try:
                temp = ser.readline()
                float(temp)
                break
            except:
                pass

        for k in range(0, int(duration/dt)):
            urnie.servoj(poses[k], control_time=dt, lookahead_time=0.2)
            data[k] = task.read()
            times[k] = time.time() - t0
            while time.time() - t0 < dt*(k+1):
                continue

    urnie.movel(startingpose, acc=0.02, vel=0.02)

    # Save data
    np.save('rand/rawdata/response_oscm_'+str(i), data)
    np.save('rand/rawdata/poses_oscm_'+str(i), poses)
    np.save('rand/rawdata/times_oscm_'+str(i), times)
    np.save('rand/rawdata/xy_oscm_'+str(i), xy)
    np.save('rand/rawdata/temp_oscm_'+str(i), float(temp))

    print(i)

urnie.close()
