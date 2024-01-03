

import py_totg
import numpy as np
import matplotlib.pyplot as plt

print("Py_totg version: ", py_totg.__version__)

def calculate_acceleration(time_step, vel):
	acc = np.zeros(vel.shape)
	for i in range(vel.shape[0]-1):
		acc[i+1, :] = (vel[i+1, :] - vel[i, :]) / time_step
	return acc


# setup
vel = np.array([1.0, 1.0, 1.0]) * 8
acc = np.array([1.0, 1.0, 1.0]) * 8
time_step = 0.01
max_deviation = 0.02

PyTrajectory = py_totg.ToTg(3, time_step, vel, acc)

print("Maximal velocity: ", PyTrajectory.get_max_velocity())
print("Maximal acceleration: ", PyTrajectory.get_max_acceleration())

# waypoints
waypoints = [
	[0.0, 0.0, 0.0],
	[0.0, 0.2, 1.0],
	[0.0, 3.0, 0.5],
	[1.1, 2.0, 0.0],
	[1.0, 0.0, 0.0],
	[0.0, 1.0, 0.0],
	[0.0, 0.0, 1.0]
]

PyTrajectory.set_waypoints(waypoints, max_deviation)
PyTrajectory.compute_trajectory()

# get results
res_pos = np.array(PyTrajectory.get_res_position())
res_vel = np.array(PyTrajectory.get_res_velocity())
res_acc = calculate_acceleration(time_step, res_vel)

# plot
t = np.arange(0, res_pos.shape[0]*time_step, time_step)
fig, (ax1, ax2, ax3) = plt.subplots(3)
ax1.plot(t, res_pos)
ax1.set_ylabel('position')
ax1.legend('123', title='DOF #')
ax2.plot(t, res_vel)
ax2.set_ylabel('velocity')
ax2.legend('123', title='DOF #')
ax3.plot(t, res_acc)
ax3.set_ylabel('acceleration')
ax3.legend('123', title='DOF #')
plt.show()
