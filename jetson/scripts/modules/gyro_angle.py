import time



t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from mpu9250_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue
z_offset = -0.018
start_angle = 0
current_angle = 0
previous_time = time.time() * 1000
current_time = time.time() * 1000
while True:
    if start_bool==False: # make sure the IMU was started
        print("IMU not Started, Check Wiring") # check wiring if error
        break
    ##################################
    # Reading and Printing IMU values
    ##################################
    #
    try:
        
        _,_,_,_,_,wz = mpu6050_conv() # read and convert mpu6050 data
        # mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        current_time = time.time() * 1000
        current_angle += (wz + z_offset)* ((current_time - previous_time)/1000)
        # print(wz)
        previous_time = current_time
        print(current_angle)
    except KeyboardInterrupt:
        print("Break!")
        break
    except:
        continue 