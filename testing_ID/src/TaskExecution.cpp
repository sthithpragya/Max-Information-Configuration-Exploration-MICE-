#include <testing_ID/TaskExecution.h>
#include <testing_ID/tauPrediction_SVR.hpp>
#include <testing_ID/helperFunctions.hpp>

TaskExecution::TaskExecution(ros::NodeHandle &n,                   // Constructor
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
    std::vector<double> betaInit,
    std::vector<double> thetaInit,
    std::vector<double> kInit,
    std::vector<double> phInit,
    std::vector<double> torquePropGain,
    std::vector<double> torqueDerGain,
    std::string learningMethod,
    std::string robotJointState,
    std::string positionController,
    std::string torqueController,
    std::string modelDirectory,
    std::string meanDataFileName,
    std::string stdDataFileName,
    std::string urdf_string):


    _loopRate(pubFreq),
    _prevRobotState(qStart, qDotStart, std::vector<double> (totalJoints, 0)),
    _currentRobotState(qStart, qDotStart, std::vector<double> (totalJoints, 0)),
    _desiredRobotState(qStart, qDotStart, std::vector<double> (totalJoints, 0)){

        // imported from param.yaml
                
        _totalJoints = totalJoints;
        _posStat = posStat;
        _velStat = velStat;
        _accStat = accStat;
        _pubFreq = pubFreq;
        _frequencyPeriod = frequencyPeriod;
        _qError = qError;
        _qDotError = qDotError;
        _qdDotError = qdDotError;
        _useFB = useFB;
        _useFF = useFF;
        _gravComp = gravComp;
        _useErrorModel = useErrorModel;
        _dynamicExecution = dynamicExecution;
        _realTimePred = realTimePred;
        _varFreq = varFreq;
        _gravCompMethod = gravCompMethod;
        _qStart = qStart;
        _qDotStart = qDotStart;
        _qLimUpper = qLimUpper;
        _qLimLower = qLimLower;
        _qDotLimUpper = qDotLimUpper;
        _qDotLimLower = qDotLimLower;
        _beta = betaInit;
        _theta = thetaInit;
        _k = kInit;
        _ph = phInit;
        _torquePropGain = torquePropGain;    // Kp
        _torqueDerGain = torqueDerGain;     // Kd
        _learningMethod = learningMethod;
        _robotJointState = robotJointState;
        _positionController = positionController;
        _torqueController = torqueController;
        _modelDirectory = modelDirectory;
        _meanDataFileName = meanDataFileName;
        _stdDataFileName = stdDataFileName;
        _urdf_string = urdf_string;
        _dT = 1/pubFreq;
        _DT = DT;

        _iterCount = 0;
        _targetArrivalTime = 0.0;
        _targetPoints = 0;

        // Getting mean and std of the train data set
        _meanData.clear();
        std::fstream meanDataTxt(modelDirectory+"/"+meanDataFileName);
        std::string line;

        while (std::getline(meanDataTxt, line)){   
            std::stringstream ss(line);
            double value;
            ss >> value;
            _meanData.push_back(value);
        }

        _stdData.clear();
        std::fstream stdDataTxt(modelDirectory+"/"+stdDataFileName);
        std::string stdline;

        while (std::getline(stdDataTxt, stdline)){
            std::stringstream ss(stdline);
            double value;
            ss >> value;
            _stdData.push_back(value);
        }

        // Initialisation of other variables
        _tauPredDataNormalised = std::vector<double> (totalJoints, 0.0);
        _tauPredData = std::vector<double> (totalJoints, 0.0);
        _gravCompPredData = std::vector<double> (totalJoints, 0.0);
        _URDFgravCompTau = std::vector<double> (totalJoints, 0.0);
        _FFtau = std::vector<double> (totalJoints, 0.0);
        _currentJointEff = std::vector<double> (totalJoints, 0.0);
        
        _gravCompTau = std::vector<double> (totalJoints, 0.0);
        _computedTau = std::vector<double> (totalJoints, 0.0);
        _computedPos = qStart;
        _FFtauNN = std::vector<double> (totalJoints, 0.0);
        _FFGravCompTauNN = std::vector<double> (totalJoints, 0.0);
        _FBtau = std::vector<double> (totalJoints, 0.0);

        _propFBtau.resize(totalJoints);
        _derFBtau.resize(totalJoints);
        
        rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);
        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        rbdyn_urdf.mbc.gravity = {0.0, 0.0, -9.81};

        _rbd_indices.clear();
        for (size_t i = 0; i < rbdyn_urdf.mb.nrJoints(); i++) {
            if (rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed){
                _rbd_indices.push_back(i);
            }
        }

        // Loading SVR models
        std::string modelName;
        if(useErrorModel){
            modelName = "error";
        }
        else{
            modelName = "tau";
        }
        for(int modelIndex = 0; modelIndex < totalJoints; modelIndex++){
            
            std::string currentTrainedModelFile = modelDirectory + "/" + modelName + std::to_string(modelIndex) + ".dat.model";
            const char *MODEL_FILE = currentTrainedModelFile.c_str();
            
            struct svm_model* SVMModel;
            if ((SVMModel = svm_load_model(MODEL_FILE)) == 0) {
                ROS_INFO_STREAM_NAMED("Can't load SVM model %s", MODEL_FILE);
            }
            _trainedModels.push_back(SVMModel);
        }
    }



bool TaskExecution::init(){
    // Subscribers
    _currentJointInformation = _n.subscribe<sensor_msgs::JointState> (_robotJointState, 100, &TaskExecution::updateRobotState, this, ros::TransportHints().reliable().tcpNoDelay());
    _getNNPrediction = _n.subscribe<sensor_msgs::JointState>("/nnPrediction", 100, &TaskExecution::updateNNPredTorque, this, ros::TransportHints().reliable().tcpNoDelay());

    // Publishers
    _pubTau = _n.advertise<std_msgs::Float64MultiArray>(_torqueController, 1);
    _pubDesPos = _n.advertise<std_msgs::Float64MultiArray>(_positionController, 1);
    _pubDesPosRec = _n.advertise<sensor_msgs::JointState>("/desPoseBreakup", 1);
    _pubTauRec = _n.advertise<sensor_msgs::JointState>("/torqueBreakup", 1);
    _timeTracker = _n.advertise<std_msgs::Float64>("/ElapsedTimeTracker", 1);
    _nnDataPub = _n.advertise<sensor_msgs::JointState>("/nnData", 1);

    // Diagnostics
    if (_n.ok()){ 
        // Wait for poses being published
        ros::spinOnce();
        ROS_INFO("Testing started");
        return true;
    }
    else{
        ROS_ERROR("The ros node has a problem");
        return false;
    }
}

void TaskExecution::run(){
    _tNow = ros::Time::now();
    _tInit = ros::Time::now();

    while(ros::ok()){
        mainLoop();
        ros::spinOnce();
        _loopRate.sleep();
    }
}


void TaskExecution::updateNNPredTorque(sensor_msgs::JointState msgData){
    if(_learningMethod == "ANN"){
        _FFtauNN = msgData.position;
        if(_gravCompMethod == "ANN"){
            _FFGravCompTauNN = msgData.velocity;
        }
    }
}

void TaskExecution::updateRobotState(const sensor_msgs::JointState::ConstPtr& msg){
    
    _prevRobotState = _currentRobotState;

    std::vector<double> newVel = msg->velocity;
    Eigen::VectorXd newVelEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(newVel.data(), newVel.size());
    Eigen::VectorXd currentVelEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_currentRobotState.jointVel.data(), _currentRobotState.jointVel.size());
    Eigen::VectorXd newAccEigen = (newVelEigen-currentVelEigen)*_pubFreq;
    
    _currentRobotState.jointAcc = std::vector<double> (newAccEigen.data(), newAccEigen.data() + newAccEigen.rows() * newAccEigen.cols());
    _currentRobotState.jointVel = newVel;
    _currentRobotState.jointPos = msg->position;

    _currentJointEff = msg->effort;
}


void TaskExecution::predictURDFGravComp(){

    ros::ServiceClient _iiwa_gravity_client = _n.serviceClient<iiwa_tools::GetGravity>("/iiwa/iiwa_gravity_server");
    iiwa_tools::GetGravity _gravity_srv;

    _gravity_srv.request.joint_angles.resize(_totalJoints);
    _gravity_srv.request.joint_velocities.resize(_totalJoints);
    _gravity_srv.request.joint_torques.resize(_totalJoints);
    _gravity_srv.request.gravity = {0.0, 0.0, -9.81};
    
    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        _gravity_srv.request.joint_angles[jointIndex] = _currentRobotState.jointPos[jointIndex];
        _gravity_srv.request.joint_velocities[jointIndex] = _currentRobotState.jointVel[jointIndex];
        _gravity_srv.request.joint_torques[jointIndex] = _currentJointEff[jointIndex];
    }

    _iiwa_gravity_client.call(_gravity_srv);
    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        _URDFgravCompTau[jointIndex] = _gravity_srv.response.compensation_torques[jointIndex];
    }
}

void TaskExecution::preparePredictionData(){
    
    // Preparing the desired joint state
    
    // During dynamic execution, use the sinusoid to be trackes
    if(_dynamicExecution){

        // If using variable frequency for sinusoid to be tracked
        if(_varFreq){
            if(_timeElapsed.data > _iterCount*_frequencyPeriod){
                _iterCount = _iterCount+1;
                
                vector<double> KNew(_totalJoints);
                vector<double> phNew(_totalJoints);
                KNew = _k;
                phNew = _ph;
                
                for (int i = 0; i < _totalJoints; i=i+1) {

                    KNew[i] = 0.1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.5)));
                
                    double phNewTemp1 = (_k[i] - KNew[i])*_timeElapsed.data + _ph[i];
                    double phNewTemp2 = M_PI - (_k[i] - KNew[i])*_timeElapsed.data - _ph[i];

                    double cOld = cos(_k[i]*_timeElapsed.data + _ph[i]);
                    double cNew1 = cos(KNew[i]*_timeElapsed.data + phNewTemp1);
                    double cNew2 = cos(KNew[i]*_timeElapsed.data + phNewTemp2);

                    if(cOld*cNew1 < 0){
                        phNew[i] = phNewTemp2;
                    }
                    else{
                        phNew[i] = phNewTemp1;
                    }
                }

                _k = KNew;
                _ph = phNew;

            }
        }

    _desiredRobotState.updateState(
        sinusoidPos(_timeElapsed.data, _k, _ph, _totalJoints, _qLimLower, _qLimUpper),
        sinusoidVel(_timeElapsed.data, _k, _ph, _totalJoints, _qLimLower, _qLimUpper, _qDotLimLower, _qDotLimUpper),
        sinusoidAcc(_timeElapsed.data, _k, _ph, _totalJoints, _qLimLower, _qLimUpper));
    }
    // During static execution, go to random stationary poses
    else{
        if(inrange(_currentRobotState.jointPos, _desiredRobotState.jointPos, _qError)){
            if(inrange(_currentRobotState.jointVel, _desiredRobotState.jointVel, _qDotError) && inrange(_currentRobotState.jointAcc, _desiredRobotState.jointAcc, _qdDotError)){
                _targetArrivalTime += _dT;
                if(_targetArrivalTime >= 0.1){
                    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
                        float newQ = _qLimLower[jointIndex] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(_qLimUpper[jointIndex]-_qLimLower[jointIndex])));
                        _desiredRobotState.jointPos[jointIndex] = newQ;
                        _desiredRobotState.jointVel[jointIndex] = 0.0;
                        _desiredRobotState.jointAcc[jointIndex] = 0.0;
                    }
                    _targetPoints += 1;
                    _targetArrivalTime = 0.0;
                }
            }
        }
    }
    
    // Getting the feedback gains

    for (int jointIndex = 0; jointIndex<_totalJoints; jointIndex++){


        double qError = _currentRobotState.jointPos[jointIndex] - _desiredRobotState.jointPos[jointIndex];
        double qDotError = _currentRobotState.jointVel[jointIndex] - _desiredRobotState.jointVel[jointIndex];

        _propFBtau(jointIndex) = -_torquePropGain[jointIndex]*qError;
        _derFBtau(jointIndex) = -_torqueDerGain[jointIndex]*qDotError;
        
        double posData;
        switch (_posStat){
            case 1:
                posData = _currentRobotState.jointPos[jointIndex];
                break;
            case 2: 
                posData = _desiredRobotState.jointPos[jointIndex];
                break;
            case 3: 
                posData = _currentRobotState.jointPos[jointIndex] + _desiredRobotState.jointVel[jointIndex]*_DT;
                break;
            
            default:
                posData = _desiredRobotState.jointPos[jointIndex];
                ROS_INFO("Warning: posStat missing. Using desired joint angle for torque prediction");
            break;
        }

        double velData;
        switch (_velStat){
            case 0:
                velData = 0.0;
                break;
            case 1:
                velData = _currentRobotState.jointVel[jointIndex];
                break;
            case 2: 
                velData = _desiredRobotState.jointVel[jointIndex];
                break;
        
            default:
                velData = _desiredRobotState.jointVel[jointIndex];
                ROS_INFO("Warning: velStat missing. Using desired joint vel for torque prediction");
            break;
        }

        double accData;
        switch (_accStat){
            case 0:
                accData = 0.0;
                break;
            case 1: 
                accData = _currentRobotState.jointAcc[jointIndex];
                break;
            case 2: 
                accData = _desiredRobotState.jointAcc[jointIndex];
                break;
            case 3: 
                accData = (_desiredRobotState.jointVel[jointIndex] - _currentRobotState.jointVel[jointIndex])/_DT;
                break;

            default:
                accData = _desiredRobotState.jointAcc[jointIndex];
                ROS_INFO("Warning: accStat missing. Using desired joint acc for torque prediction");
            break;
        }


        // Data to predict the feedforward torque (posData, velData, accData)
        _tauPredData[jointIndex] = posData;
        _tauPredData[jointIndex+_totalJoints] = velData;       
        _tauPredData[jointIndex+2*_totalJoints] =  accData;
        
        // Data to predict the feedforward torque (posData, velData, accData) (normalised)
        _tauPredDataNormalised[jointIndex] = (posData - _meanData[jointIndex])/_stdData[jointIndex];
        _tauPredDataNormalised[jointIndex+_totalJoints] = (velData - _meanData[jointIndex+_totalJoints])/_stdData[jointIndex+_totalJoints];       
        _tauPredDataNormalised[jointIndex+2*_totalJoints] =  (accData - _meanData[jointIndex+2*_totalJoints])/_stdData[jointIndex+2*_totalJoints];

        // Data to predict the gravity compensation torque (currentJointPos, 0 vel, 0 acc) - May not necessarily be used
        _gravCompPredData[jointIndex] = (_currentRobotState.jointPos[jointIndex] - _meanData[jointIndex])/_stdData[jointIndex];
        _gravCompPredData[jointIndex+_totalJoints] = (0.0 - _meanData[jointIndex+_totalJoints])/_stdData[jointIndex+_totalJoints];       
        _gravCompPredData[jointIndex+2*_totalJoints] =  (0.0 - _meanData[jointIndex+2*_totalJoints])/_stdData[jointIndex+2*_totalJoints];
    }

    // Send data to NN node if using NN based predictions
    if(_learningMethod == "ANN"){
        _nnData.position = _tauPredDataNormalised;

        if(_gravCompMethod == "ANN"){
            _nnData.velocity = _gravCompPredData;
        }

        _nnDataPub.publish(_nnData);
    }
}

void TaskExecution::getFBtau(){
    // Getting the feedback torques
    Eigen::VectorXd tempFBtauEig = _propFBtau + _derFBtau;
    _FBtau = std::vector<double> (tempFBtauEig.data(), tempFBtauEig.data() + tempFBtauEig.rows() * tempFBtauEig.cols());
}

void TaskExecution::getFFtau(rbd::InverseDynamics invDyn){
    // get FF tau

    // Getting the feedforward torques (from the URDF)
        // Used as is if the learning method set to URDF
        // Supplemented with model predictions if using Error models
    if(_useErrorModel || _learningMethod == "URDF"){
        int jIndex = 0;
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i]; // working joint index

            rbdyn_urdf.mbc.q[rbd_index][0] = _tauPredData[jIndex];
            rbdyn_urdf.mbc.alpha[rbd_index][0] = _tauPredData[jIndex + _totalJoints];
            rbdyn_urdf.mbc.alphaD[rbd_index][0] = _tauPredData[jIndex + 2*_totalJoints];
            jIndex = jIndex + 1;
        }

        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        invDyn.inverseDynamics(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        // feedforward torques as computed by the URDF
        _FFtau.clear();
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i]; // working joint index
            _FFtau.push_back(-rbdyn_urdf.mbc.jointTorque[rbd_index][0]); // (-) since urdf computes the reaction torques
        }
    }
    // Else, using NN or SVR based Full model or Error model
    else{
        for(int i = 0; i < _totalJoints; i++){
            // Using the Error model
            if(_useErrorModel){ // supplement the URDF computed feedforward torques with model predicted errors

                if(_learningMethod == "nuSVR"){
                    _FFtau[i] = _FFtau[i] + predictedTau(_tauPredDataNormalised, _trainedModels[i]);
                }
                else if(_learningMethod == "ANN"){
                    _FFtau[i] = _FFtau[i] + _FFtauNN[i];
                }
            }
            // Using the full model
            else{ 
                if(_learningMethod == "nuSVR"){
                    _FFtau[i] = predictedTau(_tauPredDataNormalised, _trainedModels[i]);
                }

                else if(_learningMethod == "ANN"){
                    _FFtau[i] = _FFtauNN[i];
                }
            }
        }
    }
}

void TaskExecution::getGravCompTau(rbd::InverseDynamics invDyn){
    // get gravity compensation cancellation torques

    // Getting the torques (from the URDF)
        // Used as is if the learning method set to URDF
        // Supplemented with model predictions if using Error models
    if(_useErrorModel || _gravCompMethod == "URDF"){
        int jIndex = 0;
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i]; // working joint index

            rbdyn_urdf.mbc.q[rbd_index][0] = _tauPredData[jIndex];
            rbdyn_urdf.mbc.alpha[rbd_index][0] = 0.0;
            rbdyn_urdf.mbc.alphaD[rbd_index][0] = 0.0;
            jIndex = jIndex + 1;
        }

        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        invDyn.inverseDynamics(rbdyn_urdf.mb, rbdyn_urdf.mbc);

        // feedforward torques as computed by the URDF
        _gravCompTau.clear();
        for (size_t i = 0; i < _rbd_indices.size(); i++) {
            size_t rbd_index = _rbd_indices[i]; // working joint index
            _gravCompTau.push_back(-rbdyn_urdf.mbc.jointTorque[rbd_index][0]); // (-) since urdf computes the reaction torques
        }
    }

    // Else, using NN or SVR based Full model or Error model
    else{
        for(int i = 0; i < _totalJoints; i++){
            // Using the Error model
            if(_useErrorModel){ // supplement the URDF computed feedforward torques with model predicted errors
                if(_gravCompMethod == "nuSVR"){
                    _gravCompTau[i] = _gravCompTau[i] + predictedTau(_gravCompPredData, _trainedModels[i]);
                }
                else if(_learningMethod == "ANN"){
                    _gravCompTau[i] = _gravCompTau[i] + _FFGravCompTauNN[i];
                }
            }
            // Using the full model
            else{ 
                if(_learningMethod == "nuSVR"){
                    _gravCompTau[i] = predictedTau(_gravCompPredData, _trainedModels[i]);
                }
                else if(_learningMethod == "ANN"){
                    _gravCompTau[i] = _FFGravCompTauNN[i];
                }
            }
        }
    }
}

void TaskExecution::updateTimeElapsed(){
    double prevTimeInstance = _tNow.toSec();
    _tNow = ros::Time::now();
    _dT = _tNow.toSec() - prevTimeInstance; // dt computed as the time gap between previous and current instancc. Do not confuse with DT which is a separate parameter
    if(_dT > 0.6){
        _dT = 0.005;
    }
    
    _timeElapsed.data = _tNow.toSec() - _tInit.toSec();
    std::cout<<"Time elapsed:   "<<_timeElapsed.data<<std::endl;
}

void TaskExecution::updateDataToRecord(){
    _tauRec.position = _FFtau;
    _tauRec.velocity = _FBtau;
    _tauRec.effort = _gravCompTau;
}

void TaskExecution::updateCommandTorque(){
    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){            
        _computedTau[jointIndex] = 0.0;

        if(_useFB){
            _computedTau[jointIndex] += _FBtau[jointIndex];
        }
        if(_useFF){
            _computedTau[jointIndex] += _FFtau[jointIndex];
        }
        if(_gravComp){
            _computedTau[jointIndex] += _gravCompTau[jointIndex];
        }
    }

    _tauCommand.data = _computedTau;
}

void TaskExecution::updateCommandPos(){
    
    _posCommand.data.clear();
    _posRec.position.clear();
    _posRec.velocity.clear();
    _posRec.effort.clear();
    
    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        _posCommand.data.push_back(_desiredRobotState.jointPos[jointIndex]);
        
        _posRec.position.push_back(_desiredRobotState.jointPos[jointIndex]);
        _posRec.velocity.push_back(_desiredRobotState.jointVel[jointIndex]);
        _posRec.effort.push_back(_desiredRobotState.jointAcc[jointIndex]);
    }
}



void TaskExecution::publishData(){
    // Publish elapsed time
    updateTimeElapsed();
    _timeTracker.publish(_timeElapsed);

    // Publish torques to recording node
    updateDataToRecord();
    _pubTauRec.publish(_tauRec);

    updateCommandTorque();
    updateCommandPos();

    // Publish positions to recording node
    _pubDesPosRec.publish(_posRec);

    // During dynamic execution, publish the command torques
    if(_dynamicExecution){
        _pubTau.publish(_tauCommand);
    }
    // During static execution, publish the command position
    
    else{        
        _pubDesPos.publish(_posCommand);
    }
}

void TaskExecution::info(){
    std::string gravCompURDFStr, gravCompModelStr, FFTauStr, FBTauStr, finalTauStr, jointEffStr;

    gravCompURDFStr = " ";
    gravCompModelStr = " ";
    FFTauStr = " ";
    FBTauStr = " ";
    finalTauStr = " ";
    jointEffStr = " ";

    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        gravCompURDFStr += " | " + std::to_string(_URDFgravCompTau[jointIndex]);
        gravCompModelStr += " | " +  std::to_string(_gravCompTau[jointIndex]);
        FFTauStr += " | " +  std::to_string(_FFtau[jointIndex]);
        FBTauStr += " | " +  std::to_string(_FBtau[jointIndex]);
        finalTauStr += " | " +  std::to_string(_computedTau[jointIndex]);
        jointEffStr += " | " +  std::to_string(_currentJointEff[jointIndex]);
    }

    std::cout<<"GravComp URDF :"<<gravCompURDFStr<<std::endl;
    std::cout<<"GravComp model:"<<gravCompModelStr<<std::endl;
    std::cout<<"FFTau         :"<<FFTauStr<<std::endl;
    std::cout<<"FBTau         :"<<FBTauStr<<std::endl;
    std::cout<<"Final Tau     :"<<finalTauStr<<std::endl;
    std::cout<<"JointEff      :"<<jointEffStr<<std::endl;
    std::cout<<"    "<<std::endl;
}


void TaskExecution::mainLoop(){

    rbd::InverseDynamics _invDyn(rbdyn_urdf.mb);
    preparePredictionData();
    predictURDFGravComp();
    getFBtau();
    getFFtau(_invDyn);
    getGravCompTau(_invDyn);
    publishData();
    info();
}