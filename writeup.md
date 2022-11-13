## Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

The standard deviation was determined by LibreOffice using the following formula between the first (`B1`) and the last lines (`1812`)
```excel
=stdev(B1:1812)
```
Which happenend to be 0.4897 for acceleration and 0.7093 for GPS.

![Sensor Noise](./images/06_sensornoise2.png)

Success criteria: 
 ### &check; The standard deviations should accurately capture the value of approximately ~68% of the respective measurements 

![Sensor Noise](./images/06_sensornoise.png)


## Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

### &check; The improved integration scheme results in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation.

A Nonlinear Complementary Filter was utilized according to the refernced whitepaper [Estimation for Quadrotors](https://www.overleaf.com/project/5c34caab7ecefc04087273b9) 7.1.2



```cpp
auto yawPitchRollQuaternion = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
yawPitchRollQuaternion.IntegrateBodyRate(gyro, dtIMU);

float predictedPitch = yawPitchRollQuaternion.Pitch();
float predictedRoll = yawPitchRollQuaternion.Roll();
ekfState(6) = yawPitchRollQuaternion.Yaw();


// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
else if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
```

![Attitude Estimation](./images/07_AttitudeEst.png)

## Implement all of the elements of the prediction step for the estimator.

![Predict State](./images/08_predictState.png)

This step just advances the predicted state by the way of simple integration
```cpp
predictedState(0) = curState(0) + dt * curState(3);
predictedState(1) = curState(1) + dt * curState(4);
predictedState(2) = curState(2) + dt * curState(5);

V3F accelI (attitude.Rotate_BtoI(accel));

predictedState(3) = curState(3) + dt * accelI.x;
predictedState(4) = curState(4) + dt * accelI.y;
predictedState(5) = curState(5) + dt * accelI.z - dt * CONST_GRAVITY;
```
## Implement the magnetometer update

This is a straight forward implementation as it is a direct reding of the measured yaw; the only thing to make sure is that the reading is properly clamped between -2 $\pi$ and +2 $\pi$ and.

Again since this is linear, the derivative h'(x_t) is a matrix of zeros and ones: 
```cpp
MatrixXf hPrime(1, QUAD_EKF_NUM_STATES);
hPrime(6) = 1;
```

![Magnetometer Update](./images/10_MagUpdate.png)  
### &check; error should be the short way around, not the long way

## Implement the GPS update


### &check; The simulation has completed the entire simulation cycle with estimated position error of < 1m.
