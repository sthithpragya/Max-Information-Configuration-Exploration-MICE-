#include <data_collection/HLPolarDS.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "High-level_PolarDS");
    ros::NodeHandle n;

    std::string publishToRobot;
    std::string robotJointState; 
    std::vector<double> qStart;
    std::vector<double> qDotStart;
    std::vector<double> qLimUpper;
    std::vector<double> qLimLower;
    std::vector<double> qDotLimUpper;
    std::vector<double> qDotLimLower;
    std::vector<double> rLim;
    int totalJoints;
    int pubFreq;
    double dT;
    double alpha;
    double beta;
    double randPointCount;
    double increm;
    double angleDispLimit;
    std::string looping;

    n.getParam("/publishToRobot", publishToRobot);
    n.getParam("/robotJointState", robotJointState);
    n.getParam("/qStart", qStart);
    n.getParam("/qDotStart", qDotStart);
    n.getParam("/qLimUpper", qLimUpper);
    n.getParam("/qLimLower", qLimLower);
    n.getParam("/qDotLimUpper", qDotLimUpper);
    n.getParam("/qDotLimLower", qDotLimLower);
    n.getParam("/rLim", rLim);
    n.getParam("/totalJoints", totalJoints);
    n.getParam("/pubFreq", pubFreq);
    n.getParam("/dT", dT);
    n.getParam("/alpha", alpha);
    n.getParam("/beta", beta);
    n.getParam("/randPointCount", randPointCount);
    n.getParam("/increm", increm);
    n.getParam("/angleDispLimit", angleDispLimit);
    n.getParam("/looping", looping);

    HLPolarDS trajectoryGen(n, publishToRobot, robotJointState, 
            qStart, qDotStart, qLimUpper, qLimLower, qDotLimUpper, qDotLimLower, 
            rLim, totalJoints, 
            pubFreq, 
            dT, alpha, beta, randPointCount, increm, 
            angleDispLimit, looping);

    if (!trajectoryGen.init()){
        return -1;
    }
    else{
        trajectoryGen.run();
    }

    return 0;











}

