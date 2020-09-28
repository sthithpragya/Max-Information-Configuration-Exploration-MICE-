#include <RBDyn/FD.h>
#include <RBDyn/ID.h>
#include <SpaceVecAlg/Conversions.h>
#include <mc_rbdyn_urdf/urdf.h>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <ros/console.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/IDIM.h>
using namespace std;

vector<vector<double>> parse2DCsvFile(string inputFileName) {

    vector<vector<double> > data;
    ifstream inputFile(inputFileName);
    int l = 0;
 
    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<double> record;
 
            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
    }
 
    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }
    return data;
}

vector<vector<double>> jointStateData; //  only joint data q, qdot, qddot
vector<vector<double>> idealTauData; // tau0, tau1, ....tau6
vector<sva::ForceVecd> idealTau;
string urdf_string, full_param;
string robot_description = "/robot_description";
string end_effector;
string jointDataFileName;
string idealTauFileName;
string regressorYFileName;
vector<size_t> _rbd_indices;
Eigen::MatrixXd yMatrix;
bool testSetPrep;
// bool grouped;

int totalJoints;
vector<double> jointState;

int main (int argc, char** argv){
    ros::init(argc, argv, "inverseDynCalc");
    ros::NodeHandle nh("~");
    ros::NodeHandle _n;
    
    nh.getParam("/savePath", jointDataFileName);
    nh.getParam("/totalJoints", totalJoints);
    nh.getParam("testSetPrep", testSetPrep);


    if(testSetPrep){
        idealTauFileName = jointDataFileName + "/InvDynPredTauData_test.csv";
        regressorYFileName = jointDataFileName + "/regressorYData_test.csv";
        jointDataFileName = jointDataFileName + "/testJointData_filtered.csv";
    }
    else{
        idealTauFileName = jointDataFileName + "/InvDynPredTauData_train.csv";
        regressorYFileName = jointDataFileName + "/regressorYData_train.csv";
        jointDataFileName = jointDataFileName + "/trainJointData_filtered.csv";
    }

    ofstream idealTauFile(idealTauFileName);
    ofstream regressorYFile(regressorYFileName);

    // gets the location of the robot description on the parameter server
    if (!_n.searchParam(robot_description, full_param)) {
        ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
        return false;
    }

    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
        ROS_INFO_ONCE_NAMED("Controller", "Controller is waiting for model"
                                                        " URDF in parameter [%s] on the ROS param server.",
            robot_description.c_str());
        _n.getParam(full_param, urdf_string);
        usleep(100000);
    }

    // ROS_INFO("full param is %s", full_param.c_str());
    ROS_INFO("urdf string is %s", urdf_string.c_str());

    ROS_INFO_STREAM_NAMED("Controller", "Received urdf from param server, parsing...");

    jointStateData = parse2DCsvFile(jointDataFileName);

    mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string); // initialising the rbd
    // mc_rbdyn_urdf::URDFParserResult rbdyn_urdf_fd = rbdyn_urdf;

    rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
    rbdyn_urdf.mbc.gravity = {0.0, 0.0, -9.81};
    rbd::InverseDynamics invDyn(rbdyn_urdf.mb);
    rbd::IDIM idimDyn(rbdyn_urdf.mb);

    int timeIndex = 0;
    int endIndex = jointStateData.size();

    _rbd_indices.clear();
    for (size_t i = 0; i < rbdyn_urdf.mb.nrJoints(); i++) {
        if (rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
            _rbd_indices.push_back(i);
    }

    // updating the rbdyn_urdf
    ros::Rate rate(100000);
    while(ros::ok()){

        ROS_INFO("timeIndex %d", timeIndex);
        // ROS_INFO("timeIndex %d", timeIndex);
        // ROS_INFO("%s", regressorYFileName.c_str());


        if(timeIndex < endIndex){
            jointState = jointStateData[timeIndex];
            timeIndex = timeIndex + 1;
            ROS_INFO("vector data %f %f %f %f %f %f", jointState[0], jointState[1], jointState[2], jointState[18], jointState[19], jointState[20]);


            int jointIndex = 0;

            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                size_t rbd_index = _rbd_indices[i]; // working joint index
                rbdyn_urdf.mbc.q[rbd_index][0] = jointState[jointIndex];

                rbdyn_urdf.mbc.alpha[rbd_index][0] = jointState[jointIndex + totalJoints];
                rbdyn_urdf.mbc.alphaD[rbd_index][0] = jointState[jointIndex + 2*totalJoints];
                jointIndex = jointIndex + 1;            
            }


            rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
            rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
            invDyn.inverseDynamics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
            idimDyn.computeY(rbdyn_urdf.mb, rbdyn_urdf.mbc);

            yMatrix = idimDyn.Y(); 
            
            // write taus to another file
            for (size_t i = 0; i < _rbd_indices.size(); i++) {
                size_t rbd_index = _rbd_indices[i]; // working joint index
                ROS_INFO("joint torque %lu %f",rbd_index, rbdyn_urdf.mbc.jointTorque[rbd_index][0]);

                idealTauFile << rbdyn_urdf.mbc.jointTorque[rbd_index][0] << ",";
            }
            idealTauFile << '\n';

            
            for(int  i = 0; i < yMatrix.rows(); i++){
                for(int j = 0; j < yMatrix.cols(); j++){
                    string str = to_string(yMatrix(i,j));
                    if(j+1 == yMatrix.cols()){
                        regressorYFile<<str;
                    }
                    else{
                        regressorYFile<<str<<',';
                    }
                }
            regressorYFile<<'\n';
            }
        }
        else{
            // close the file and exit
            idealTauFile.close();
            regressorYFile.close();
            ROS_INFO("Inverse Dynamics computation complete");
            ROS_INFO("RegressorY computation complete");
            exit (EXIT_FAILURE);
        }

        ros::spinOnce();
        rate.sleep();
    }
}