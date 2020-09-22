#ifndef __HLPOLARDS_H__
#define __HLPOLARDS_H__

#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include "Eigen/Dense"
#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <time.h>
#include <string>
#include <typeinfo>
#include <cmath>
#include <algorithm> 
#include <cstdlib>
#include <ctime>

class HLPolarDS{

    private:
        /** 
         * --------------------------------------
         * IMPORTED FROM PARAM.YAML
         * --------------------------------------
        **/

            // Topic names 
                std::string _publishToRobot;
                std::string _robotJointState;

            // Vectors
                std::vector<double> _qStart;
                std::vector<double> _qDotStart;
                std::vector<double> _qLimUpper;
                std::vector<double> _qLimLower;
                std::vector<double> _qDotLimUpper;
                std::vector<double> _qDotLimLower;
                std::vector<double> _rLim;
            
            // Ints and Doubles
                int _totalJoints;
                double _pubFreq;
                double _dT;
                double _alpha;
                double _beta;
                double _randPointCount;
                double _increm;
                double _angleDispLimit;

            // Other Strings
                std::string _looping;

        /** 
         * --------------------------------------
         * USED DURING TRAJECTORY GENERATION
         * --------------------------------------
        **/

            // Ints and Doubles
                int _haltCount;                             // Count of joints which have completed their cycles
                int _targetCount;                           // Count of cycles each joint has completed

            // Vectors
                std::vector<double> _angleDisp;
                std::vector<int> _cyclePointCount;          // Points swept in transit around one eq point
                std::vector<bool> _switchCycle;             // Time to check for target completion or not
                std::vector<bool> _haltCycle;               // Stop joint's motion as cycle has been successfully completed,
                                                            // wait for other joints to finish as well

            // Eigens                
                std::vector<double> _qDes;                      // initialise to qStart
                std::vector<double> _qDotDes;                   // initialise to qDotStart
                
                std::vector<double> _qCurrent;                  // initialise to qStart
                std::vector<double> _qDotCurrent;               // initialise to qDotStart

                std::vector<double> _qNext;                     // initialise to qStart
                std::vector<double> _qDotNext;                  // initialise to qDotStart

                Eigen::MatrixXd _paramGridLengthMat;


            // ROS Parameters
                ros::NodeHandle _n;
                ros::Rate _loopRate;
                ros::Time _tNow;
                ros::Time _tInit;
                std_msgs::Float64MultiArray _commandPose;
                std_msgs::Float64MultiArray _commandVel;
                std_msgs::Float64 _timeElapsed;             // Elapsed time

            // SUBSCRIBER declaration
                ros::Subscriber _currentJointInformation;   // Listen to joint states

            // PUBLISHER declaration
                ros::Publisher _pubCommandPose;             // Publish command position
                ros::Publisher _pubCommandVel;              // Publish command velocity
                ros::Publisher _pubTimeElapsed;             // Publish time elapsed

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

        struct Phi{
            Eigen::VectorXd r;
            Eigen::VectorXd c;

            Phi(){
                r = Eigen::VectorXd(0);
                c = Eigen::VectorXd(0);
            }


            Phi(Eigen::VectorXd rVal, Eigen::VectorXd cVal){
                r = rVal;
                c = cVal;
            }

            void updatePhi(Phi phiVal){
                r.resize(phiVal.r.rows(), phiVal.r.cols());
                c.resize(phiVal.c.rows(), phiVal.c.cols());

                r = phiVal.r;
                c = phiVal.c;

            }
        };

        struct ParamGrid{
            std::vector<double> rs;
            std::vector<double> cs;

            ParamGrid(){
                rs = std::vector<double> {0.0};
                cs = std::vector<double> {0.0};
            }

            ParamGrid(std::vector<double> rsVal, std::vector<double> csVal){
                rs = rsVal;
                cs = csVal;
            }

            void updateGrid(double rsVal, double csVal){
                rs.push_back(rsVal);
                cs.push_back(csVal);
            }

            void updateGrid(std::vector<double> rsVal, std::vector<double> csVal){
                rs.insert(rs.end(), rsVal.begin(), rsVal.end());
                cs.insert(cs.end(), csVal.begin(), csVal.end());
            }

            void updateGrid(double rsVal, std::vector<double> csVal){
                std::vector<double> rsTemp(csVal.size(), rsVal);
                rs.insert(rs.end(), rsTemp.begin(), rsTemp.end());
                cs.insert(cs.end(), csVal.begin(), csVal.end());
            }

        };

        struct Pdfs{

            std::vector<Phi> mu;
            Eigen::VectorXd sigma;
            double w;

            Pdfs(std::vector<Phi> muVal, Eigen::MatrixXd sigmaVal, double wVal){
                mu = muVal;
                sigma = sigmaVal;
                w = wVal;
            }

            Pdfs(){
                mu = std::vector<Phi> {Phi()};
                sigma =  Eigen::VectorXd::Zero(1);;
                w = 1;
            }

            void updatePdfs(Phi muVal){
                mu.push_back(muVal);
            }
        };

        RobotState _prevRobotState;                       // Keep track of manipulator's previos position
        RobotState _currentRobotState;                    // Keep track of robot's current position

        Phi _phiDes = {};                                      // Desired (r, c)
        
        std::vector<ParamGrid> _paramGrid;                // Vector of Grid containing (r, c) combinations over the parameter space of each joint
        Pdfs _pdfs = {};                                       // Distribution data associated with each cycle  

        /** 
         * --------------------------------------
         * METHODS
         * --------------------------------------
        **/

            HLPolarDS(ros::NodeHandle &n,                   // Constructor
                std::string publishToRobot, 
                std::string robotJointState, 
                std::vector<double> qStart,
                std::vector<double> qDotStart,
                std::vector<double> qLimUpper,
                std::vector<double> qLimLower,
                std::vector<double> qDotLimUpper,
                std::vector<double> qDotLimLower,
                std::vector<double> rLim,
                int totalJoints,
                double pubFreq,
                double dT,
                double alpha,
                double beta,
                double randPointCount,
                double increm,
                double angleDispLimit,
                std::string looping);                      

            bool init();                                    // Initialize node
            void run();                                     // Run node
            void updateTimeElapsed();                       // timeElapsed = tNow - tInit

            void updateRobotState(const sensor_msgs::JointState::ConstPtr& msg);                    
                                                        /** 
                                                            *  To update current and previous robot state wrt joint_state topic data
                                                            *      Update prevJointPos (wrt old currentJoint data)
                                                            *      Update prevJointVel   
                                                            *      Update prevJointAcc   
                                                            *      Update currentJointPos wrt message data
                                                            *      Update currentJointVel wrt message data
                                                            *      Calculate the jointAcc and update currentJointAcc 
                                                        **/

            void updateCommandPose();                       // Updates the _commandPose to be published to the robot
                                                            // ~ next_point()

            void publishData();                             // Publishes data to the topics (first calls updateCommandPose() and updateTimeElapsed())

            void makeParamGrid();                           // Prepares the (r,c) combination grid

            void mainLoop();                                // Main loop responsible for exploring the parameter space

            void checkSwitchCycle();                        // Tracks the progress of joint's motion around target for completion
            void checkHaltCycle();                          // Tracks the progress of joint's motion around target for completion

            void getNextPoint();                            // Next (q, qDot) joint configuration to trace the cycle around target
    
            void getNextTarget();                           // Target to cycle around next
    
            Phi new_target_smart();
            std::vector<Phi> optimiser(Phi phiTemp);


            Phi getDSparam(std::vector<double> qT, std::vector<double> qDotT, double rT);
            double mvnpdf(Phi x, Phi mu);



};



#endif