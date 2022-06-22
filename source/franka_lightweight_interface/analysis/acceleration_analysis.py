from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import plotly.express as px
from scipy.signal import savgol_filter
from scipy import interpolate
import scipy
from mpl_toolkits.mplot3d import axes3d

# Load dataset
data = np.loadtxt("analysis/data_accel.txt").T

time = data[0]
time = (time - time[0])*1e-6
position = data[1:8, :]
control_torque = data[8:15, :]
measured_torque = data[15:22, :]
gravity_torque = data[22:29, :]
coriolis_torque = data[29:36, :]
mass = data[36:, :]

# Filter out noise and estimate position derivatives
measured_torque = savgol_filter(measured_torque, 111, 3)

velocity = savgol_filter(position, window_length=111, polyorder=2, deriv=1, delta=1e-3)
acceleration = savgol_filter(position, window_length=351, polyorder=2, deriv=2, delta=1e-3)

# Estimate model torque from acceleration and mass matrix
est_torque = np.zeros_like(control_torque)*np.nan
est_acceleration = np.zeros_like(control_torque)*np.nan
for i in range((est_torque.shape[1])):
    new_mass = mass[:, i].reshape((7,7))
    est_torque[:, i] = new_mass@acceleration[:, i]
    est_acceleration[:, i] = scipy.linalg.solve(new_mass, control_torque[:, i], assume_a='pos')

# Resample dataset at lower frequency and regular spaced time grid
f_position = interpolate.interp1d(time, position, axis=1)
f_velocity = interpolate.interp1d(time, velocity, axis=1)
f_acceleration = interpolate.interp1d(time, acceleration, axis=1)
f_measured_torque = interpolate.interp1d(time, measured_torque, axis=1)
f_control_torque = interpolate.interp1d(time, control_torque, axis=1)
f_est_torque = interpolate.interp1d(time, est_torque, axis=1)
f_est_acceleration = interpolate.interp1d(time, est_acceleration, axis=1)

dT = 50e-3
time = np.arange(time[0], time[-1], dT)

position = f_position(time)
velocity = f_velocity(time)
acceleration = f_acceleration(time)
measured_torque = f_measured_torque(time)
control_torque = f_control_torque(time)
est_torque = f_est_torque(time)
est_acceleration = f_est_acceleration(time)



# ------------------------- Plot ---------------------- #

# Time serie
fig, axs = plt.subplots(4, 2)
for i, ax in enumerate(axs.ravel()[:-1]):
    ax.plot(time, control_torque[i, :], label = "Control")
    ax.plot(time, measured_torque[i, :], label="Measured")
    ax.plot(time, est_torque[i, :], label="Estimated")

    ax.legend(loc='upper right', prop={'size': 6})
    # ax.set(ylabel="joint {} [degree]".format(i))
fig.suptitle("Joint torques [N.m] over time [s]")

# Time serie
fig, axs = plt.subplots(4, 2)
for i, ax in enumerate(axs.ravel()[:-1]):
    ax.plot(time, acceleration[i, :], label = "Measured by differentiation")
    ax.plot(time, est_acceleration[i, :], label="Estimated from libfranka model")

    ax.legend(loc='upper right', prop={'size': 6})
    # ax.set(ylabel="joint {} [degree]".format(i))
fig.suptitle("Joint acceleration [rad/s2] over time [s]")



# fig = plt.figure()

# for i in range(2):

#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')
#     # ax = fig.add_subplot(4, 2, i+1, projection='3d')
#     ax.scatter(position[i, :], velocity[i, :], control_torque[i, :]-est_torque[i, :], "+")
#     ax.set_xlabel("Position")
#     ax.set_ylabel("Velocity")

# fig, axs = plt.subplots(4, 2)
# for i, ax in enumerate(axs.ravel()[:-1]):
#     ax.plot(time, position[i, :], label = "pos")
#     ax.plot(time, velocity[i, :], label="vel")
#     ax.plot(time, acceleration[i, :], label="acc")

#     ax.legend(loc='upper left')

# plt.plot(est_torque[6, :], measured_torque[6, :], "+")



plt.show()