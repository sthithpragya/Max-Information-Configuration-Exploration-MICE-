#ifndef __TASKEXECUTION_H__
#define __TASKEXECUTION_H__

#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <time.h>
#include <string>
#include <typeinfo>
#include <vector>
#include "libSVMpackage/svm.h"
#include <fstream>
#include <cstdlib>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FD.h>
#include <RBDyn/ID.h>
#include "Eigen/Dense"
#include <iiwa_tools/iiwa_tools.h>
#include <iiwa_tools/GetGravity.h>
#include <stdio.h> 
#include <stdlib.h> 

class TaskExecution{

    private:
        /** 
         * --------------------------------------
         * IMPORTED FROM PARAM.YAML
         * --------------------------------------
        **/
           
           // Ints and Doubles
                int _totalJoints;
                int _posStat;
                int _velStat;
                int _accStat;
                double _pubFreq;
                double _frequencyPeriod;
                double _qError;
                double _qDotError;
                double _qdDotError;

            // Bools
                bool _useFB;
                bool _useFF;
                bool _gravComp;
                bool _useErrorModel;
                bool _dynamicExecution;
                bool _realTimePred;

                bool _varFreq;
                bool _testNow;

            // Vectors
                std::vector<double> _qStart;
                std::vector<double> _qDotStart;
                std::vector<double> _qLimUpper;
                std::vector<double> _qLimLower;
                std::vector<double> _qDotLimUpper;
                std::vector<double> _qDotLimLower;

                std::vector<double> _beta;
                std::vector<double> _theta;
                std::vector<double> _k;
                std::vector<double> _ph;

                std::vector<double> _torquePropGain;    // Kp
                std::vector<double> _torqueDerGain;     // Kd

                std::vector<std::vector<double>> _randPoses; // _totalJoints x 200 dimensional vector containing joint angles for stationary poses 

            // Strings
                std::string _learningMethod;
                std::string _gravCompMethod;
                // std::string robot_description = "/robot_description";

            // Topic names 
                std::string _robotJointState;
                std::string _positionController;
                std::string _torqueController;
                std::string _modelDirectory;

                std::string _meanDataFileName;
                std::string _stdDataFileName;

        /** 
         * --------------------------------------
         * USED DURING EXECUTION
         * --------------------------------------
        **/ 

            // Ints and Doubles
                int _iterCount;                         // Trackes changes in sinusoidal trajectories frequency
                int _targetPoints;                      // Random positions explored during static execution
                double _targetArrivalTime;              // Time spent per pose
                
                double _dT;                             // time between two spins ~ 1/pubfreq
                double _DT;                             // user specified interval

            // Vectors

                std::vector<double> _meanData;
                std::vector<double> _stdData;

                std::vector<double> _tauPredDataNormalised; // joint-state data to be used for predicting feedforward torque (normalised)
                std::vector<double> _tauPredData;       // joint-state data to be used for predicting feedforward torque

                std::vector<double> _gravCompPredData;  // joint-state data to be used for predicting gravity compensation torques
                std::vector<double> _gravCompTau;       // Gravity compensation cancellation torques
                std::vector<double> _URDFgravCompTau;   // Gravity compensation cancellation torques (compuuted using the URDF)

                std::vector<double> _computedTau;       // final computed joint torques
                std::vector<double> _computedPos;       // final joint position


                std::vector<double> _FBtau;             // cummulative feedback torque
                std::vector<double> _FFtau;             // Feedforward torque predicted by the model
                std::vector<double> _FFtauNN;           // NN predicted FF torque (or error)
                std::vector<double> _FFGravCompTauNN;   // NN predicted gravity compensatio
                std::vector<double> _currentJointEff;   // Current joint torques

                // std::vector<double> gravCompTorque;     // torque to negate internally applied gravity cancellation torques

                // extern std::vector<struct svm_model*> _trainedModels;  // SVM models
                std::vector<struct svm_model*> _trainedModels;  // SVM models
                std::vector<size_t> _rbd_indices;

            // Strings
                std::string _urdf_string; 
                std::string _full_param;


            // Eigens
                Eigen::VectorXd _propFBtau;         // fb torque due to proportional gains
                Eigen::VectorXd _derFBtau;          // fb torque due to derivate gains


            // ROS Parameters
                ros::Time _tNow;
                ros::Time _tInit;
                ros::NodeHandle _n;
                ros::Rate _loopRate;

                std_msgs::Float64 _timeElapsed;         // Elapsed time
                sensor_msgs::JointState _tauRec;        // Breakup of overall tau wrt ff, fb and gravComp
                std_msgs::Float64MultiArray _tauCommand;// Command torque to be published during dynamic execution
                sensor_msgs::JointState _nnData;        // Robot joint state data for node making the NN prediction

                std_msgs::Float64MultiArray _posCommand;// Command pos to be published during static execution
                sensor_msgs::JointState _posRec;        // Breakup of overall position wrt joint angle and velocity

                mc_rbdyn_urdf::URDFParserResult rbdyn_urdf;

            // SUBSCRIBER declaration
                ros::Subscriber _currentJointInformation;   // Joint data from robot sensors
                ros::Subscriber _getNNPrediction;           // Prediction from the node running the NN

            // PUBLISHER DECLARATION
                ros::Publisher _pubTau;      // publishes overall torque = Feedforward + FeedBack + GravComp
                ros::Publisher _pubTauRec;   // publishes overall torque as {Feedforward, FeedBack, GravComp}
                ros::Publisher _pubDesPos;       // publishes the desired joint pos
                ros::Publisher _pubDesPosRec;    // records the desired joint pos
                ros::Publisher _timeTracker;    
                ros::Publisher _nnDataPub;       // publishes joint data for Neural networks (if used) to make predictions                

    public:
        struct RobotState{

            std::vector<double> jointPos;
            std::vector<double> jointVel;
            std::vector<double> jointAcc;

            RobotState(){
            }

            RobotState(std::vector<double> currentJointPos, std::vector<double> currentJointVel, std::vector<double> currentJointAcc){
                
                jointPos = currentJointPos;
                jointVel = currentJointVel;
                jointAcc = currentJointAcc;
            }

            void updateState(std::vector<double> currentJointPos, std::vector<double> currentJointVel, std::vector<double> currentJointAcc){
                
                jointPos = currentJointPos;
                jointVel = currentJointVel;
                jointAcc = currentJointAcc;
            }
        };

        RobotState _prevRobotState;                       // Keep track of manipulator's previos position
        RobotState _currentRobotState;                    // Keep track of robot's current position
        RobotState _desiredRobotState;                    // Keep track of robot's current position


    /** 
         * --------------------------------------
         * METHODS
         * --------------------------------------
    **/ 

        TaskExecution(ros::NodeHandle &n,                   // Constructor
                int totalJoints,
                int posStat,
                int velStat,
                int accStat,
                double frequencyPeriod,
                double qError,
                double qDotError,
                double qdDotError,
                double DT,
                double pubFreq,
                bool useFB,
                bool useFF,
                bool gravComp,
                bool useErrorModel,
                bool dynamicExecution,
                bool realTimePred,
                bool varFreq,
                std::string gravCompMethod,
                std::vector<double> qStart,
                std::vector<double> qDotStart,
                std::vector<double> qLimUpper,
                std::vector<double> qLimLower,
                std::vector<double> qDotLimUpper,
                std::vector<double> qDotLimLower,
                std::vector<double> beta,
                std::vector<double> theta,
                std::vector<double> k,
                std::vector<double> ph,
                std::vector<double> torquePropGain,
                std::vector<double> torqueDerGain,
                std::string learningMethod,
                std::string robotJointState,
                std::string positionController,
                std::string torqueController,
                std::string modelDirectory,
                std::string meanDataFileName,
                std::string stdDataFileName,
                std::string urdf_string,
                bool _testNow);


        bool init();                                    // Initialize node
        void run();                                     // Run node

        void updateNNPredTorque(sensor_msgs::JointState msgData);
        void updateRobotState(const sensor_msgs::JointState::ConstPtr& msg);

        void publishData();                             // Publishes data to the topics (first calls updateCommandPose() and updateTimeElapsed())

        void preparePredictionData();                   // Sorts data required for making predictions
        void predictURDFGravComp();                     // computing URDF gravity compensation
        
        
        void getFBtau();                                // Get the feddback torques
        void getFFtau(rbd::InverseDynamics invDyn);     // Gets the predicted joint torques i.e. update the commanded torque data
        void getGravCompTau(rbd::InverseDynamics invDyn);// Gets the joint torques to cancel internally applied gravity compensation

        void updateTimeElapsed();                       // timeElapsed = tNow - tInit
        void updateDataToRecord();                      // Data to record = {FF torques, FB torques, gravity compensation cancellation toqrues}
        void updateCommandTorque();
        void updateCommandPos();
        void info();
        void mainLoop();                                // Main loop responsible for starting the testing

        void generateRandPoses();                       // Generates a collection of random poses that can be used during static execution





};


#endif