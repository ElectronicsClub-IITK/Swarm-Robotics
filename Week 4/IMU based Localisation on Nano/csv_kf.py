import pandas as pd
import numpy as np
from pykalman import KalmanFilter
import matplotlib.pyplot as plt

data = pd.read_csv('accelerometer_data.csv')
timestamps = data['timestamp'].values
acc_data = data[['acc_x', 'acc_y', 'acc_z']].values
gyro_data = data[['gyro_x', 'gyro_y', 'gyro_z']].values

initial_state_mean = [acc_data[0, 0], acc_data[0, 1], acc_data[0, 2], gyro_data[0, 0], gyro_data[0, 1], gyro_data[0, 2]]
transition_matrix = np.eye(6)
observation_matrix = np.eye(6)
process_noise_covariance = np.eye(6) * 1e-5
measurement_noise_covariance = np.eye(6) * 1e-1

kf = KalmanFilter(
    transition_matrices=transition_matrix,
    observation_matrices=observation_matrix,
    initial_state_mean=initial_state_mean,
    transition_covariance=process_noise_covariance,
    observation_covariance=measurement_noise_covariance
)

observations = np.hstack((acc_data, gyro_data))
filtered_state_means, filtered_state_covariances = kf.filter(observations)

filtered_acc = filtered_state_means[:, :3]
filtered_gyro = filtered_state_means[:, 3:]

filtered_data = np.hstack((filtered_acc, filtered_gyro))
filtered_df = pd.DataFrame(filtered_data, columns=['filtered_acc_x', 'filtered_acc_y', 'filtered_acc_z', 'filtered_gyro_x', 'filtered_gyro_y', 'filtered_gyro_z'])
print(filtered_df)

plt.figure(figsize=(15, 20))

for i, axis in enumerate(['x', 'y', 'z']):
    plt.subplot(3, 2, 2*i + 1)
    plt.plot(timestamps, acc_data[:, i], 'r', label=f'Raw acc_{axis}')
    plt.plot(timestamps, filtered_acc[:, i], 'b', label=f'Filtered acc_{axis}')
    plt.legend()
    plt.title(f'Accelerometer Data (axis: {axis})')

for i, axis in enumerate(['x', 'y', 'z']):
    plt.subplot(3, 2, 2*i + 2)
    plt.plot(timestamps, gyro_data[:, i], 'r', label=f'Raw gyro_{axis}')
    plt.plot(timestamps, filtered_gyro[:, i], 'b', label=f'Filtered gyro_{axis}')
    plt.legend()
    plt.title(f'Gyroscope Data (axis: {axis})')

plt.tight_layout()
plt.show()
