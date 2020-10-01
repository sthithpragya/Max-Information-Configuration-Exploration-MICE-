_______________________________________________________________________________________________

InvDyn_Learning Package:
_______________________________________________________________________________________________

	Contains 4 sub-packages:
		1) data_collection
		2) data_processing
		2*) data_processing_extended
		3) learning
		4) testing_ID
		5) error_computation (requires *)
_______________________________________________________________________________________________

Introduction:
_______________________________________________________________________________________________

1) data_collection: 
	> Trajectory generator responsible for phase space exploration.
	> Based on the MICE approach
	> Published the desired joint-pose to the robot in open-loop position control
	> Records joint-data during robot-motion

2) data_processing:
	> Removes the noise and repeated entries (if any) from data
	> Computes joint-accelarations
	> Subsamples the data and prepared training and test sets

2*) data_processing_extended:
	> Same functionality as data_processing
	> Also implements the relevance score-based subsampling strategy
	> Also has provision to fit GMM to data and compute the sequential batch-wise entropies
	> Also provision to plot entropies of different datasets

3) learning:
	> Implements parametric least squared regression method to find optimal physical parameters of the robot
	> Implements nuSVR, ANN Coupled, ANN Decoupled to learn the Torque and Error models

4) testing_ID:
	> Tests the learnt models for predicting torques during static and dynamic execution
	> Prepares the mean squared errors in torque prediction during static execution 
	> Prepares the mean squared errors in position and velocity tracking during dynamic execution

5) error_computation:
	> Computes the errors between the Torque precition by the ideal RBD model and the real-world joint torques
_______________________________________________________________________________________________

General flow of Program:
_______________________________________________________________________________________________

	> 'data_collection' to collect data
	> 'data_processing' to clean and process the data
	> 'error_computation' to get the torque prediction errors (on the processed data)
	> 'data_processing' again to prepare train and test sets
	> 'learning' to learn the Torque and Error models
	> 'testing_ID' to test the efficacy of the learned models

_______________________________________________________________________________________________

Prerequisites:
_______________________________________________________________________________________________

	> ROS - ROS: tested in Melodic and Kinetic
	> SpaceVecAlg
	> ROS Control *
	> RBDyn *
	> mc_rbdyn_urdf *
	> corrade *
	> robot_controllers *
	
_______________________________________________________________________________________________

Preliminary steps:
_______________________________________________________________________________________________

	> Make a directory 'recorded_data' to store the recorded and processed data for learning
	> Make a directory 'learnt_models' to store the learnt models of inverse dynamics
	> Place the 'data_collection' and 'testing_id' packages inside the catkin workspace

The package specific information is available as README files in the respective packages.