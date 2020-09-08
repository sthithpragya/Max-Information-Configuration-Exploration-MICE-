edit location of yaml file in:
	mainDir/catkin_ws/src/iiwa_ros/iiwa_data_collection/scripts/recorderPositionController.py
	
configure the parameters in the yaml file at:

	mainDir/catkin_ws/src/iiwa_ros/iiwa_data_collection/config/param.yaml (for data collection)

	mainDir/data_for_learning/data_processing/param.yaml (for data processing)

	mainDir/learning (for the learning process)
		


Launching data collection (from catkin_ws):
	>roslaunch iiwa_data_collection starter.launch
		wait for the robot to reach the start(or home) configuration
	>roslaunch iiwa_data_collection PolarDS.launch

Starting data processing (mainDir/data_for_learning/data_processing)
	> Execute prepareData.sh
	> Execute selectData.sh

Learning (mainDir/learning/eSVR_libSVM):
	> change the hyperparameters in trainModels.cpp 
	> ./trainModels.sh

	> Testing
		> ./testModels.sh

Testing the learned Inverse Dynamics:
	> change the hyperparameters (more importantly modelDirectory) in mainDir/catkin_ws/src/iiwa_ros/iiwa_testing_id/config


Package Information:
1) learning - support for learning models (Torque and error models) via:
	> nuSVR (uses thunderSVM library)
	> NN (coupled dynamics' NN implemented using PyTorch)
	> NN (decoupled dynamics' NN implemented using PyTorch)

2) recorded_data - directory to store the data recorded while following the excitation trajectory

3) data_processing - scripts to sub_sample the data





Central Parameters (central_param.yaml):
> dataRecordLoc           - path where data is recorded (OR) 
							path of data which needs to be processed (OR)
							path of data to be used for learning

> dataCollectionParamLoc  - location of param.yaml used in data collection

> dataProcessParamLoc     - location of param.yaml used in data processing/ subsampling

> learningParamLoc        - location of param.yaml used during learning the Torque and Error models

> testingParamLoc         - location of param.yaml used in testing the learned inverse dynamics
