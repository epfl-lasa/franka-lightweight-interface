from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

data = np.loadtxt("analysis/data_accel.txt").T

time = data[0]
time = (time - time[0])*1e-6
position = data[1:8, :]
model_accel = data[8:15, :]
measured_accel = data[15:22, :]

measured_accel = savgol_filter(measured_accel, 111, 3)

# velocity = np.gradient(position, time, axis=1)
# velocity = savgol_filter(velocity, 111, 3)

# est_accel = np.gradient(velocity, time, axis=1)
# est_accel = savgol_filter(est_accel, 111, 3)

velocity = savgol_filter(position, window_length=111, polyorder=3, deriv=1, delta=1e-3)

est_accel = savgol_filter(position, window_length=111, polyorder=3, deriv=2, delta=1e-3)

accel_error = model_accel - measured_accel

# Time serie
fig, axs = plt.subplots(4, 2)
for i, ax in enumerate(axs.ravel()[:-1]):
    ax.plot(time, model_accel[i, :], label = "Model")
    ax.plot(time, measured_accel[i, :], label="Measured")
    ax.plot(time, est_accel[i, :], label="Estimated")

    ax.legend()
    # ax.set(ylabel="joint {} [degree]".format(i))

fig, axs = plt.subplots(4, 2)
for i, ax in enumerate(axs.ravel()[:-1]):
    ax.plot(velocity[i, :], accel_error[i, :], "+")

# fig, axs = plt.subplots(4, 2)
# for i, ax in enumerate(axs.ravel()[:-1]):
#     ax.plot(time, position[i, :], label = "pos")
#     ax.plot(time, velocity[i, :], label="vel")
#     ax.plot(time, est_accel[i, :], label="acc")

#     ax.legend()


plt.figure()
# plt.plot(np.diff(time), "+")
plt.plot(time, position.T)



plt.show()