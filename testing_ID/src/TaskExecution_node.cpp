#include <testing_ID/TaskExecution.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "Testing Inverse Dynamics");
    ros::NodeHandle n;

    int totalJoints;
    int posStat;
    int velStat;
    int accStat;
    double frequencyPeriod;
    double qError;
    double qDotError;
    double qdDotError;
    double DT;
    double pubFreq;
    bool useFB;
    bool useFF;
    bool gravComp;
    bool useErrorModel;
    bool dynamicExecution;
    bool realTimePred;
    bool varFreq;
    bool testNow;
    std::string gravCompMethod;
    std::vector<double> qStart;
    std::vector<double> qDotStart;
    std::vector<double> qLimUpper;
    std::vector<double> qLimLower;
    std::vector<double> qDotLimUpper;
    std::vector<double> qDotLimLower;
    std::vector<double> betaInit;
    std::vector<double> thetaInit;
    std::vector<double> kkInit;
    std::vector<double> phInit;
    std::vector<double> torquePropGain;
    std::vector<double> torqueDerGain;
    std::string learningMethod;
    std::string robotJointState;
    std::string positionController;
    std::string torqueController;
    std::string modelDirectory;
    std::string meanDataFileName;
    std::string stdDataFileName;

    std::string urdf_string, full_param;
    std::string robot_description = "/robot_description";


    n.getParam("/totalJoints", totalJoints);
    n.getParam("/posStat", posStat);
    n.getParam("/velStat", velStat);
    n.getParam("/accStat", accStat);
    n.getParam("/frequencyPeriod", frequencyPeriod);
    n.getParam("/qError", qError);
    n.getParam("/qDotError", qDotError);
    n.getParam("/qdDotError", qdDotError);
    n.getParam("/DT", DT);
    n.getParam("/useFB", useFB);
    n.getParam("/useFF", useFF);
    n.getParam("/gravComp", gravComp);
    n.getParam("/useErrorModel", useErrorModel);
    n.getParam("/dynamicExecution", dynamicExecution);
    n.getParam("/realTimePred", realTimePred);
    n.getParam("/varFreq", varFreq);
    n.getParam("/gravCompMethod", gravCompMethod);
    n.getParam("/qStart", qStart);
    n.getParam("/qDotStart", qDotStart);
    n.getParam("/qLimUpper", qLimUpper);
    n.getParam("/qLimLower", qLimLower);
    n.getParam("/qDotLimUpper", qDotLimUpper);
    n.getParam("/qDotLimLower", qDotLimLower);
    n.getParam("/betaInit", betaInit);
    n.getParam("/thetaInit", thetaInit);
    n.getParam("/kInit", kkInit);
    n.getParam("/phInit", phInit);
    n.getParam("/pubFreq", pubFreq);
    n.getParam("/torquePropGain", torquePropGain);
    n.getParam("/torqueDerGain", torqueDerGain);
    n.getParam("/learningMethod", learningMethod);
    n.getParam("/robotJointState", robotJointState);
    n.getParam("/positionController", positionController);
    n.getParam("/torqueController", torqueController);
    n.getParam("/modelDirectory", modelDirectory);
    n.getParam("/meanDataFileName", meanDataFileName);
    n.getParam("/stdDataFileName", stdDataFileName);
    n.getParam("/testNow", testNow);


    if (!n.searchParam(robot_description, full_param)) {
        ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
        return false;
    }

    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
        ROS_INFO_ONCE_NAMED("Controller", "Controller is waiting for model"
                                                        " URDF in parameter [%s] on the ROS param server.",
            robot_description.c_str());
        n.getParam(full_param, urdf_string);
        usleep(100000);
    }

    TaskExecution testingID(n, 
    totalJoints,posStat,velStat,accStat,
    frequencyPeriod,qError,qDotError,qdDotError,DT,pubFreq,
    useFB,useFF,gravComp,useErrorModel,dynamicExecution,realTimePred,varFreq,
    gravCompMethod,qStart,qDotStart,qLimUpper,qLimLower,qDotLimUpper,qDotLimLower,
    betaInit,thetaInit,kkInit,phInit,torquePropGain,torqueDerGain,
    learningMethod,robotJointState,positionController,torqueController,
    modelDirectory,meanDataFileName,stdDataFileName,urdf_string,testNow);

    if (!testingID.init()){
        return -1;
    }
    else{
        testingID.run();
    }
    return 0;
}

