# Filters
Filtering Robots perceive the world through sensors, but sensors are inherently noisy making it challenging to observe environmental states completely. There is advance understanding of physical systems like kinematics and dynamics of different robots. This knowledge can provide state estimates, but mathematical models too are not perfect. The problem is as follows, given noisy sensor measurements and knowledge of the system, how to combine this information such that the results is better than the individual results. A well-known solution to this problem is the use of filter algorithms to suppress sensor noise and or fuse multiple sensors for a better state estimate. There are many filters, but the famous ones in robotics are kalman filter, complementary filter, low-pass filter, band-pass filter and high-pass filter. Although named differently, all these filters share a common foundation. Here, we will derive a generic filter.
1. Assuming a point robot has have two noisy 1-D position sensors, a less accurate sensor
A with variance 2 81 m and a more accurate sensor B with variance 2 16 m . If at timet
sensor A reads 160 mand B167 m , what is the interval of the likely robot position?

Two sensors have different variance and different position reading depending on the accuracy.
The MATLAB file is attached here with to calculate the limits and likely robot position.
The interval is 150.14 m<= position <= 176.86 m.
Kalman Filter is used here.


2. Assuming a third even less accurate sensor C with variance 2 169 m is activated. If this
sensor gives a reading of150 m.
a. Use this additional information to find the most likely robot position.
b. Is the third (noisiest) sensor helpful? Justify your answer.

a. Refer to MATLAB file attached for the answer.
Likely robot position is 164.6844 m.
b. By comparing the likely position from 2 sensors and 3 sensors, it is clear that , the robot position is getting towards accuracy.
Hence, This demonstrates that even poor measurements only increase the precision of an estimate

3. Since sensors can be expensive, we use one or two sensors and supplement the state
estimate with system knowledge. Using Newton’s law of motion and robot velocity
(assuming no change in velocity), we can predict the future robot position.
a. Give the robot’s initial position x(0)=0 , constant velocity vel=0.2m/s and sample
time dt=1s . Use Newton’s equation of motion to predict the robot position for 100
time-samples.
b. Call the provided “robot_data.p” matlab file to return 100 position measurements
using the following command d=robot_data(100) . Plot the results together with
the results in (a). What is special about the motion model used in (a)?
c. If we use (a) alone we ignore sensor measurements, if we use (b) alone we
ignore system knowledge. Honestly, we have no reason to choose one over the
other. Instead, let us use both information by calculating a residual  z - x and
using a “scale_factor” to determine how much of the residual to add to the
prediction x_est = x + scale_factor*residual . Use this filter to estimate the robot
position.

![image](https://user-images.githubusercontent.com/43060427/130223288-a9548e50-d50b-4b4f-8290-9ef1db4b425c.png)
Figure 1 position time graph for filtered and unfiltered data

4. Use the knowledge from question (3) to filter the GPS position data in the localization
task. Plot the filtered position estimate in the environmental map

A GPS emulated in the MATLAB code gives the Position of the robot in the terms of x, y, z coordinate. The position of the robot changing each time has been traced out; Below figure shows the plot which makes one complete loop.
![image](https://user-images.githubusercontent.com/43060427/130223340-53585108-3080-418b-ab81-dc7214b76548.png)


5. Clean up the IMU signals as well.
a. Implement a low pass filters for gyroscope angular rates. Plot the filtered and
unfiltered readings on the same plot
b. Implement high pass filters for accelerometer measurements. Plot the filtered and
unfiltered accelerometer readings on the same plot
c. Incorporate the filters in the IMU localization solution and plot the environmental
map
Gyroscope gives reading of the orientation of the robot, that is the angular velocity around zaxis,
while the accelerometer gives the linear velocity in the direction of x- axis. From which
the position of the robot in x, y, z coordinate was calculated and implemented on the
MATLAB. Above figure shows the plot of position of the robot according to the reading of
the sensors in real-time making one complete loop.
Below figures show the plot for filtered and unfiltered gyroscope angular rates and
accelerometer measurements.
![image](https://user-images.githubusercontent.com/43060427/130223401-b4b04324-8c27-42c7-993f-60e203f4f8f3.png)
Filtered and unfiltered plot for accelerometer reading
![image](https://user-images.githubusercontent.com/43060427/130223427-fc6d17f4-925d-461c-8f2d-3dd9f5ad63e9.png)
Filtered and unifiltered plot for gyroscope reading
