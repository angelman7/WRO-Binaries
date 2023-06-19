import time

t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from modules.mpu9250_gyro_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue
    
current_angle = 0
previous_time = time.time() * 1000
current_time = time.time() * 1000
total = 0
for i in range(1000):
    
    _,_,_,_,_,wz = mpu6050_conv() # read and convert mpu6050 data
    total += wz
    time.sleep(0.02)
    # current_time = time.time() * 1000
    # current_angle += (wz + z_offset)* ((current_time - previous_time)/1000)
    # # h = heading()
    # # print(wz)
    # previous_time = current_time
print(total/1000)