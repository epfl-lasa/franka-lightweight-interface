from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from mpl_toolkits.mplot3d import axes3d

data = np.loadtxt("analysis/data_accel.txt").T

time = data[0]
time = (time - time[0])*1e-6
position = data[1:8, :]
control_torque = data[8:15, :]
measured_torque = data[15:22, :]
mass = data[22:, :]

measured_torque = savgol_filter(measured_torque, 111, 3)

# velocity = np.gradient(position, time, axis=1)
# velocity = savgol_filter(velocity, 111, 3)

# est_accel = np.gradient(velocity, time, axis=1)
# est_accel = savgol_filter(est_accel, 111, 3)

# TODO: resample with regular time grid to correctly use the derivative
velocity = savgol_filter(position, window_length=111, polyorder=2, deriv=1, delta=1e-3)
acceleration = savgol_filter(position, window_length=351, polyorder=2, deriv=2, delta=1e-3)

est_torque = np.zeros_like(control_torque)*np.nan
for i in range((est_torque.shape[1])):
    new_mass = mass[:, i].reshape((7,7))
    est_torque[:, i] = new_mass@acceleration[:, i]

# Time serie
fig, axs = plt.subplots(4, 2)
for i, ax in enumerate(axs.ravel()[:-1]):
    ax.plot(time, control_torque[i, :], label = "Control")
    ax.plot(time, measured_torque[i, :], label="Measured")
    ax.plot(time, est_torque[i, :], label="Estimated")

    ax.legend(prop={'size': 6})
    # ax.set(ylabel="joint {} [degree]".format(i))
fig.suptitle("Joint torques [N.m] over time [s]")


# fig = plt.figure()
# for i in range(7):
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

#     ax.legend(loc='upper right')

# plt.plot(est_torque[6, :], measured_torque[6, :], "+")



plt.show()