import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class KalmanFilter3D:
    def __init__(self, dt, process_noise_std, measurement_noise_std):
        # Time step
        self.dt = dt

        # State vector [angular_velocity_x, angular_velocity_y, angular_velocity_z, acceleration_x, acceleration_y, acceleration_z]
        self.x = np.zeros((6, 1))

        # State covariance matrix
        self.P = np.eye(6)

        # Process noise covariance matrix
        self.Q = np.eye(6) * process_noise_std**2

        # Measurement noise covariance matrix
        self.R = np.eye(6) * measurement_noise_std**2

        # Measurement matrix
        self.H = np.eye(6)

        # State transition matrix
        self.A = np.eye(6)
        self.A[0, 3] = self.dt
        self.A[1, 4] = self.dt
        self.A[2, 5] = self.dt

    def predict(self):
        # Predict the next state
        self.x = self.A @ self.x
        # Predict the next covariance
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update the state with the new measurement
        y = z.reshape(6, 1) - self.H @ self.x
        self.x = self.x + K @ y

        # Update the covariance matrix
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        return self.x.flatten()


# Usage example with plotting
if __name__ == "__main__":
    dt = 0.01  # Time step (e.g., 10ms)
    process_noise_std = 0.1
    measurement_noise_std = 0.1

    kf = KalmanFilter3D(dt, process_noise_std, measurement_noise_std)

    df_accelero = pd.read_csv("demo_saved_data/2024-06-05-16-40-18/accel-0.csv")
    df_gyro = pd.read_csv("demo_saved_data/2024-06-05-16-40-18/gyro-0.csv")

    acc_x = df_accelero["acc_x"]
    acc_y = df_accelero["acc_y"]
    acc_z = df_accelero["acc_z"]

    gyro_x = df_gyro["gyro_x"]
    gyro_y = df_gyro["gyro_y"]
    gyro_z = df_gyro["gyro_z"]

    measurements = [[0 for _ in range(0, 6, 1)] for _ in range(len(df_gyro))]
    for i in range(len(df_gyro)):
        measurements[i][0] = gyro_x[i]
        measurements[i][1] = gyro_y[i]
        measurements[i][2] = gyro_z[i]
        measurements[i][3] = acc_x[i]
        measurements[i][4] = acc_y[i]
        measurements[i][5] = acc_z[i]

    # Simulate some measurements (for example purposes)

    estimated_states = []

    for measurement in measurements:
        kf.predict()
        kf.update(np.array(measurement))
        estimated_states.append(kf.get_state())
        print("State estimate:", kf.get_state().flatten())

    estimated_states = np.array(estimated_states)
    measurements = np.array(measurements)

    # Plotting the results
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))
    time = np.arange(len(measurements)) * dt

    for i, label in enumerate(
        [
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
            "acceleration_x",
            "acceleration_y",
            "acceleration_z",
        ]
    ):
        axs[i].plot(time, measurements[:, i], "r", label="Measurements")
        axs[i].plot(time, estimated_states[:, i], "b", label="Kalman Filter Estimate")
        axs[i].set_xlabel("Time [s]")
        axs[i].set_ylabel(label)
        axs[i].legend()
        axs[i].grid()

    plt.tight_layout()
    plt.show()
