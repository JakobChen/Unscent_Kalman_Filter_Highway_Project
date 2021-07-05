## Objective of Unscented Kalman Filter Highway Project
This project aims to apply  unscented kamlman filter to archieve more accurated  result of the state estimation of the highway robot cars. The reason to choose unscented kalman filter is because of the nolinearation of radar model. In addition, unscented kalman filter is straitforward to implement. At the end of the project,  the RMSE(Root Mean Squared Error) has been  used to evaluate the performance of the sensor fusion method.




## Process  of UKF
In general the UKF folows the following steps:

* Step 1: Initialize the noise deviation, state variables, those could be treated as posterior for step 1, but prior for step2
* Step 2: Predict meean and covariance based on the prior.
* Step 3: Update the state with measurement, the result is posterior for step 3, but prior for step 1.
* Step 4: Jump into step 1 and repeat the whole steps

## Prediction 
* Generate Sigma Points
* Predict Sigma Points
* Predict Mean and Covariance

## Update
* Predict Measurements

* Update State
## Comparison between Lidar, Radar and Sensor Fusion on Lidar & Radar
In general, we could see from sensor fusion could achrieve best result in comparion of only radar and lidar measurment.
## Objective of Unscented Kalman Filter Highway Project
This project aims to apply  unscented kamlman filter to archieve more accurated  result of the state estimation of the highway robot cars. The reason to choose unscented kalman filter is because of the nolinearation of radar model. In addition, unscented kalman filter is straitforward to implement. At the end of the project,  the RMSE(Root Mean Squared Error) has been  used to evaluate the performance of the sensor fusion method.


## Process  of UKF
In general the UKF folows the following steps:

* Step 1: Initialize the noise deviation, state variables, those could be treated as posterior for step 1, but prior for step2
* Step 2: Predict meean and covariance based on the prior.
* Step 3: Update the state with measurement, the result is posterior for step 3, but prior for step 1.
* Step 4: Jump into step 1 and repeat the whole steps

## Prediction 
* Generate Sigma Points
* Predict Sigma Points
* Predict Mean and Covariance

## Update
* Predict Measurements

* Update State
## Comparison between Lidar, Radar and Sensor Fusion on Lidar & Radar
In general, we could see from sensor fusion could achrieve best result in comparion of only radar and lidar measurment. The raw measurements can be foud in the root folder name UFK_Measurements.xlsx

**Result Evaluation of the average result in 5 frames**
|          Type      |   X          |   Y          | Vx          |    Vy    |       
| -------------------| ---------    | -----        | -----       |  -----   | 
|      Radar         |   0.0578036	| 0.0890602	   |0.382172     |0.4935578 |
|     Lidar          |   0.054848	| 0.091428	   |0.36972	     |0.5223     | 
|Radar&Lidar Fusion  | **0.046688** | **0.080228** |**0.29262**	 |**0.49144**| 

