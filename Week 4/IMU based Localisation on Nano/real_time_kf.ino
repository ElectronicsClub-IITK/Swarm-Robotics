#include <Arduino_LSM6DS3.h>

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float state[6] = {0, 0, 0, 0, 0, 0}; 
float P[6][6] = {{1, 0, 0, 0, 0, 0},
                 {0, 1, 0, 0, 0, 0},
                 {0, 0, 1, 0, 0, 0},
                 {0, 0, 0, 1, 0, 0},
                 {0, 0, 0, 0, 1, 0},
                 {0, 0, 0, 0, 0, 1}};
const float Q[6][6] = {{1e-5, 0, 0, 0, 0, 0},
                       {0, 1e-5, 0, 0, 0, 0},
                       {0, 0, 1e-5, 0, 0, 0},
                       {0, 0, 0, 1e-5, 0, 0},
                       {0, 0, 0, 0, 1e-5, 0},
                       {0, 0, 0, 0, 0, 1e-5}};
const float R[6][6] = {{1e-1, 0, 0, 0, 0, 0},
                       {0, 1e-1, 0, 0, 0, 0},
                       {0, 0, 1e-1, 0, 0, 0},
                       {0, 0, 0, 1e-1, 0, 0},
                       {0, 0, 0, 0, 1e-1, 0},
                       {0, 0, 0, 0, 0, 1e-1}};
void matrixMultiply(const float a[6][6], const float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 6; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixAdd(const float a[6][6], const float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = a[i][j] + b[i][j];
    }
  }
}

void matrixSubtract(const float a[6][6], const float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = a[i][j] - b[i][j];
    }
  }
}

void printMatrix(const char* name, const float matrix[6][6]) {
  Serial.println(name);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      Serial.print(matrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
}

void printVector(const char* name, const float vector[6]) {
  Serial.print(name);
  Serial.print(": ");
  for (int i = 0; i < 6; i++) {
    Serial.print(vector[i]);
    Serial.print(" ");
  }
  Serial.println();
}

float K[6][6] = {{0}};
void kalmanUpdate(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z) {
  float z[6] = {acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z};
  float P_predict[6][6];
  matrixAdd(P, Q, P_predict);
  float S[6][6], y[6];  


  matrixAdd(P_predict, R, S);
  for (int i = 0; i < 6; i++) {
    K[i][i] = P_predict[i][i] / S[i][i];

    }

  for (int i = 0; i < 6; i++) {
    y[i] = z[i] - state[i];
  }

  for (int i = 0; i < 6; i++) {
    state[i] += K[i][i] * y[i];  
  }

  float K_P[6][6];
  matrixMultiply(K, P_predict, K_P);

  matrixSubtract(P, K_P, P);

  float velocity[3] = {0,0,0};
  for(i = 0; i<3; i++){
    velocity[i] += state[i]*(0.506);
  }

  float displacement[i]= {0,0,0};
  for(i=0; i<3; i++){
    displacement[i] += velocity[i]*(0.506) + (state[i]*0.506*0.506)/2;
  }

  Serial.println(displacement);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  unsigned long timestamp = millis(); 
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    kalmanUpdate(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

    Serial.print(timestamp);
  }
  delay(500);
  
}