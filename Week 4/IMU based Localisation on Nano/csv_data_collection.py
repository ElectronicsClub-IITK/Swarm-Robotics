import serial
import csv
import time

ser = serial.Serial('COM4', 9600)
time.sleep(2)

with open('accelerometer_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    
    writer.writerow(['timestamp','acc_x', 'acc_y', 'acc_z','gyro_x','gyro_y','gyro_z'])
    
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            
            if line:
                print(f"Read line: {line}")
                data = line.split(',')
                
                if len(data) == 7:
                    timestamp,acc_x, acc_y, acc_z,gyro_x,gyro_y,gyro_z= map(float, data)
                    writer.writerow([timestamp,acc_x, acc_y, acc_z,gyro_x,gyro_y,gyro_z])
                    
    except KeyboardInterrupt:
        print("Terminating...")
        
    finally:
        ser.close()
