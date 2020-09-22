#include <data_collection/HLPolarDS.h>
// Helper functions
std::vector<double> cart2pol(double q, double qDot){
	
	std::vector<double> inPolar;
	double theta = atan2(qDot, q);
	double rho = std::sqrt(std::pow(qDot,2.0)+std::pow(q,2.0));
	inPolar.push_back(theta);
	inPolar.push_back(rho);
	
	return inPolar;
}

std::vector<double> pol2cart(double theta, double rho){
	
	std::vector<double> inCartesian;
	double q = rho*cos(theta);
	double qDot = rho*sin(theta);	
	inCartesian.push_back(q);
	inCartesian.push_back(qDot);

	return inCartesian;
}


// Member functions of HLPolarDS
HLPolarDS::HLPolarDS(ros::NodeHandle &n, 
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
    std::string looping):
    
    _loopRate(pubFreq),
    _prevRobotState(qStart, qDotStart, std::vector<double> (totalJoints, 0)),
    _currentRobotState(qStart, qDotStart, std::vector<double> (totalJoints, 0)){

        // imported from param.yaml
        _n = n;
        _publishToRobot = publishToRobot; 
        _robotJointState = robotJointState; 
        _qStart = qStart;
        _qDotStart = qDotStart;
        _qLimUpper = qLimUpper;
        _qLimLower = qLimLower;
        _qDotLimUpper = qDotLimUpper;
        _qDotLimLower = qDotLimLower;
        _rLim = rLim;
        _totalJoints = totalJoints;
        _pubFreq = pubFreq;
        _dT = dT;
        _alpha = alpha;
        _beta = beta;
        _randPointCount = randPointCount;
        _increm = increm;
        _angleDispLimit = angleDispLimit;
        _looping = looping;
    
        // used during execution

        _qDes = qStart;
        _qDotDes = qDotStart;
        
        _qNext = std::vector<double> (totalJoints, 0.0001);
        _qDotNext = std::vector<double> (totalJoints, 0.0001);
        _qCurrent = std::vector<double> (totalJoints, 0.0001);
        _qDotCurrent = std::vector<double> (totalJoints, 0.0001);

        _phiDes.updatePhi(getDSparam(_qDes, _qDotDes, rLim[1]));

        makeParamGrid(); // Prepares _paramGrid   

        _angleDisp = std::vector<double> (totalJoints, 0.0);
        _targetCount = 0;

        _cyclePointCount = std::vector<int> (totalJoints, 0);
        _switchCycle = std::vector<bool> (totalJoints, false);
        _haltCycle = std::vector<bool> (totalJoints, false);
        
        _haltCount = 0;
        
        _commandPose.data = qStart;
        _commandVel.data = qDotStart;
        _timeElapsed.data = 0.0;

        Eigen::VectorXd _sigma(2);
        _sigma(0) = 0.0025; // sigmaR
        _sigma(1) = 0.01;   // sigmaC

        _pdfs.sigma = _sigma;
        _pdfs.mu.clear();

        std::srand (static_cast <unsigned> (time(0)));

    }

bool HLPolarDS::init(){
    // Subscribers
    _currentJointInformation = _n.subscribe<sensor_msgs::JointState> (_robotJointState, 100, &HLPolarDS::updateRobotState, this, ros::TransportHints().reliable().tcpNoDelay());

    // Publishers
    _pubCommandPose = _n.advertise<std_msgs::Float64MultiArray>(_publishToRobot, 1);
    _pubCommandVel = _n.advertise<std_msgs::Float64MultiArray>("/CommandVel", 1);
    _pubTimeElapsed = _n.advertise<std_msgs::Float64>("/ElapsedTimeTracker", 1);

    // Diagnostics
    if (_n.ok()){ 
        // Wait for poses being published
        ros::spinOnce();
        ROS_INFO("Trajectory generator is ready");
        return true;
    }
    else{
        ROS_ERROR("The ros node has a problem");
        return false;
    }
}

void HLPolarDS::run(){
    _tNow = ros::Time::now();
    _tInit = ros::Time::now();

    while(ros::ok()){
        mainLoop();
        ros::spinOnce();
        _loopRate.sleep();
    }
}

void HLPolarDS::updateTimeElapsed(){
    _tNow = ros::Time::now();
    _timeElapsed.data = _tNow.toSec() - _tInit.toSec();
    std::cout<<"Time elapsed:   "<<_timeElapsed.data<<std::endl;
}

void HLPolarDS::updateCommandPose(){
    _commandPose.data.clear();
    _commandPose.data = _qNext;
    _commandVel.data = _qDotNext;
}

void HLPolarDS::publishData(){
    updateTimeElapsed();
    updateCommandPose();
    _pubTimeElapsed.publish(_timeElapsed);
    _pubCommandPose.publish(_commandPose);
    _pubCommandVel.publish(_commandVel);
}

void HLPolarDS::updateRobotState(const sensor_msgs::JointState::ConstPtr& msg){
    
    _prevRobotState = _currentRobotState;

    std::vector<double> newVel = msg->velocity;
    Eigen::VectorXd newVelEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(newVel.data(), newVel.size());
    Eigen::VectorXd currentVelEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_currentRobotState.jointVel.data(), _currentRobotState.jointVel.size());
    Eigen::VectorXd newAccEigen = (newVelEigen-currentVelEigen)/_dT;
    
    _currentRobotState.jointAcc = std::vector<double> (newAccEigen.data(), newAccEigen.data() + newAccEigen.rows() * newAccEigen.cols());
    _currentRobotState.jointVel = newVel;
    _currentRobotState.jointPos = msg->position;
}

void HLPolarDS::checkSwitchCycle(){

    std::string switchStatus = "";
    std::string angleDispStatus = "";

    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        if(_angleDisp[jointIndex] > 2.5*M_PI){
            _switchCycle[jointIndex] = true;
            _angleDisp[jointIndex] = 0.0;
        }
        switchStatus = switchStatus + std::to_string(int(_switchCycle[jointIndex])) + "     ";
        angleDispStatus = angleDispStatus + std::to_string(_angleDisp[jointIndex]) + "     ";

    }    
    std::cout<<"Switch status:  "<<switchStatus<<std::endl;
    std::cout<<"Angle disp.:    "<<angleDispStatus<<std::endl;
}

void HLPolarDS::checkHaltCycle(){
    std::string haltStatus = "";

    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        if(_switchCycle[jointIndex]){
            if(std::fabs(_qDotCurrent[jointIndex]) < 0.005){
                _haltCycle[jointIndex] = true;
                _haltCount = _haltCount + 1;
            }
        }

        haltStatus = haltStatus + std::to_string(int(_haltCycle[jointIndex])) + "     ";
    }
    std::cout<<"Halt status:    "<<haltStatus<<std::endl;
    std::cout<<"Halt count:     "<<_haltCount<<std::endl;
}

void HLPolarDS::getNextPoint(){
   
    _qCurrent = _qNext;
    _qDotCurrent = _qDotNext;


    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        
        if(!_haltCycle[jointIndex]){

            std::vector<double> thetaRho = cart2pol(_qCurrent[jointIndex] - _phiDes.c(jointIndex), _qDotCurrent[jointIndex]);
            
            double dr = thetaRho[1] - _phiDes.r(jointIndex);
            if(std::fabs(dr) > 0.5){
                if(dr > 0){
                    dr = 0.5;
                }
                else{
                    dr = -0.5;
                }
            }

            double rDot = -_alpha*dr;
            double thetaDot = -_beta*std::exp(-std::pow(_alpha*dr,2));


            double rNew = rDot*_dT + thetaRho[1];
            double thetaNew = thetaDot*_dT + thetaRho[0];

            std::vector<double> jointState = pol2cart(thetaNew, rNew);

            jointState[0] = jointState[0] + _phiDes.c(jointIndex);

            if(std::fabs(jointState[0] - _qCurrent[jointIndex]) < 0.005){
                jointState[0] = _qCurrent[jointIndex] + _dT*jointState[1]; // with this, gets stuck
            }
            
            jointState[1] = (jointState[0] - _qCurrent[jointIndex])/_dT;

            jointState[0] = std::fmax(jointState[0], _qLimLower[jointIndex]);
            jointState[0] = std::fmin(jointState[0], _qLimUpper[jointIndex]);

            jointState[1] = std::fmax(jointState[1], _qDotLimLower[jointIndex]);
            jointState[1] = std::fmin(jointState[1], _qDotLimUpper[jointIndex]);

            _qNext[jointIndex] = jointState[0];
            _qDotNext[jointIndex] = jointState[1];
                        
            std::vector<double> thetaRhoDisp = cart2pol(jointState[0] - _phiDes.c(jointIndex), jointState[1]);
            double dTheta = std::fabs(thetaRhoDisp[0] - thetaRho[0]);
            if(dTheta > M_PI){
                dTheta = 0.0;
            }

            _angleDisp[jointIndex] = _angleDisp[jointIndex] + dTheta;
            _cyclePointCount[jointIndex] = _cyclePointCount[jointIndex] + 1;
        }
        else{
            _qNext[jointIndex] = _qCurrent[jointIndex];
            _qDotNext[jointIndex] = 0.0;
        }

    }

    std::cout<<"Target count:   "<<_targetCount<<std::endl;
    std::cout<<"\n"<<std::endl;

}

void HLPolarDS::getNextTarget(){
    if(_haltCount >= _totalJoints){
        // Reset trackers
        _targetCount = _targetCount + 1;
        _haltCount = 0;
        _switchCycle = std::vector<bool> (_totalJoints, false);
        _haltCycle = std::vector<bool> (_totalJoints, false);

        _pdfs.updatePdfs(_phiDes);  // adds (r,c) to the log
        _pdfs.w = 1.0/_targetCount;

        _phiDes.updatePhi(new_target_smart());
    }
}

HLPolarDS::Phi HLPolarDS::new_target_smart(){
    Eigen::MatrixXd randIn = Eigen::MatrixXd::Random(_totalJoints,_randPointCount);
        randIn = randIn.cwiseAbs();
        randIn = randIn.cwiseProduct(_paramGridLengthMat);

        Eigen::VectorXd maxVal = randIn.rowwise().maxCoeff();
        Eigen::VectorXd minVal = randIn.rowwise().minCoeff();

        Eigen::MatrixXd rndParamsR = Eigen::MatrixXd::Zero(_totalJoints,_randPointCount);
        Eigen::MatrixXd rndParamsC = Eigen::MatrixXd::Zero(_totalJoints,_randPointCount);

        double dr = 0.5*_increm;
        double dc = _increm;

        for(int randPointIndex = 0; randPointIndex < _randPointCount; randPointIndex++){
            for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
                int indx = randIn(jointIndex, randPointIndex);
                double randR = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                double randC = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                rndParamsR(jointIndex, randPointIndex) = _paramGrid[jointIndex].rs[indx] - dr + 2.0*dr*randR;
                rndParamsC(jointIndex, randPointIndex) = _paramGrid[jointIndex].cs[indx] - dc + 2.0*dc*randC;
            }
        }

        std::vector<HLPolarDS::Phi> phiCandidates;
        std::vector<double> yCandidates;

        for(int randPointIndex = 0; randPointIndex < _randPointCount; randPointIndex++){
            Eigen::VectorXd rndParamsR_temp = rndParamsR.col(randPointIndex);
            Eigen::VectorXd rndParamsC_temp = rndParamsC.col(randPointIndex);

            HLPolarDS::Phi _phiTemp(rndParamsR_temp, rndParamsC_temp);

            std::vector<HLPolarDS::Phi> phiOptimalCandidate = optimiser(_phiTemp);

            phiCandidates.push_back(phiOptimalCandidate[0]);
            yCandidates.push_back(phiOptimalCandidate[1].r(0));
            // return type of struct with [phi, double]
            // harvest Yopt as Phi from return
        }

        int indxF = std::max_element(yCandidates.begin(),yCandidates.end()) - yCandidates.begin();
        // selects the first minimum point

        return phiCandidates[indxF];
}

std::vector<HLPolarDS::Phi> HLPolarDS::optimiser(HLPolarDS::Phi phiTemp){
    
    double gamma = 0.005;
    double er = 0.01;
    double epsilon = 0.001;
    int nIter = 500;


    Eigen::VectorXd gradR = Eigen::VectorXd::Zero(phiTemp.r.size());
    Eigen::VectorXd gradC = Eigen::VectorXd::Zero(phiTemp.c.size());

    Eigen::MatrixXd sigmaR = Eigen::MatrixXd::Identity(_totalJoints, _totalJoints) * _pdfs.sigma(0);
    Eigen::MatrixXd sigmaC = Eigen::MatrixXd::Identity(_totalJoints, _totalJoints) * _pdfs.sigma(1);


    int iterCount = 0;
    while((iterCount < nIter) && (er > epsilon)){
        for(int j = 0; j < _pdfs.mu.size(); j++){

            double mvnpdfVal = mvnpdf(phiTemp, _pdfs.mu[j]);

            Eigen::VectorXd diffR = phiTemp.r - _pdfs.mu[j].r;
            Eigen::VectorXd diffC = phiTemp.c - _pdfs.mu[j].c;

            Eigen::VectorXd rhsR = diffR*_pdfs.w*mvnpdfVal;
            Eigen::VectorXd rhsC = diffC*_pdfs.w*mvnpdfVal;

            Eigen::VectorXd gradRtemp = sigmaR.householderQr().solve(rhsR);
            Eigen::VectorXd gradCtemp = sigmaC.householderQr().solve(rhsC);

            gradR = gradR + gradRtemp;
            gradC = gradC + gradCtemp;
        }

        HLPolarDS::Phi phiOld = phiTemp;

        phiTemp.r = phiTemp.r + gamma*gradR; 
        phiTemp.c = phiTemp.c + gamma*gradC;

        for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
            phiTemp.r(jointIndex) = std::fmax(_rLim[0], std::fmin(phiTemp.r(jointIndex), _rLim[1]));

            double du = _qLimUpper[jointIndex] - phiTemp.r(jointIndex) - _increm;
            double dl = _qLimLower[jointIndex] + phiTemp.r(jointIndex) + _increm;

            phiTemp.c(jointIndex) = std::fmax(dl, std::fmin(phiTemp.c(jointIndex), du));
        }

        Eigen::VectorXd erVec(phiTemp.r.size() + phiTemp.c.size());
        erVec << phiTemp.r - phiOld.r, phiTemp.c - phiOld.c;

        er = erVec.norm();
        iterCount = iterCount + 1;
    }

    HLPolarDS::Phi XOpt = phiTemp;
    double yOpt = 0.0;

    for(int j = 0; j < _pdfs.mu.size(); j++){
        yOpt = yOpt + _pdfs.w* mvnpdf(XOpt, _pdfs.mu[j]);
    }

    Eigen::VectorXd yOptEigen(1);
    yOptEigen << yOpt;

    HLPolarDS::Phi YOpt(yOptEigen, yOptEigen);

    std::vector<HLPolarDS::Phi> phiOptimal;
    phiOptimal.push_back(XOpt);
    phiOptimal.push_back(YOpt);
    
    return phiOptimal;
}

void HLPolarDS::mainLoop(){
    getNextPoint();
    publishData();
    
    checkSwitchCycle();
    checkHaltCycle();
    
    getNextTarget();
}


HLPolarDS::Phi HLPolarDS::getDSparam(std::vector<double> qT, std::vector<double> qDotT, double rT){
    
    std::vector<double> R(qT.size(), 0.0);
    std::vector<double> C(qT.size(), 0.0);
    
    for(int jointIndex = 0; jointIndex < qT.size(); jointIndex = jointIndex+1){

        double Ctemp = 0.0;
        double Rtemp = std::sqrt(std::pow(qT[jointIndex],2) + std::pow(qDotT[jointIndex],2));

        if(Rtemp > rT){
            if(qT[jointIndex] < 0){
                Ctemp = qT[jointIndex] + std::sqrt(std::fabs(std::pow(rT,2) - std::pow(qDotT[jointIndex],2)));
            }
            else{
                Ctemp = qT[jointIndex] - std::sqrt(std::fabs(std::pow(rT,2) - std::pow(qDotT[jointIndex],2)));
            }
            Rtemp = rT;
        }

        R[jointIndex] = Rtemp;
        C[jointIndex] = Ctemp;
    }

    Eigen::VectorXd REigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(R.data(), R.size());
    Eigen::VectorXd CEigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(C.data(), C.size());

    return HLPolarDS::Phi(REigen, CEigen);
}

void HLPolarDS::makeParamGrid(){

    std::vector<double> rList;
    std::vector<double> lengthList;

    int Rlength = 1 + int((_rLim[1] - _rLim[0] - _increm)/_increm);
    
    for(int i = 0; i < Rlength; i++){
        rList.push_back(_rLim[0] + (double(i) + 0.5)*_increm);
    }

    std::vector<ParamGrid> pGrid;

    for(int jointIndex = 0; jointIndex < _totalJoints; jointIndex++){
        
        ParamGrid pGridTemp = {};
        pGridTemp.rs.clear();
        pGridTemp.cs.clear();
        
        for(int rIndex = 0; rIndex < Rlength; rIndex++){

            double du = _qLimUpper[jointIndex] - rList[rIndex] - 0.5*_increm;
            double dl = _qLimLower[jointIndex] + rList[rIndex] + 0.5*_increm;

            std::vector<double> cList;

            int Clength = 1 + int((du - dl)/(_increm*double(2.0)));
            for(int i = 0; i < Clength; i++){
                cList.push_back(dl + double(i*2.0)*_increm);
            }

            pGridTemp.updateGrid(rList[rIndex], cList);
        }
        lengthList.push_back(pGridTemp.cs.size());
        pGrid.push_back(pGridTemp);
    }
    _paramGrid = pGrid;
    Eigen::VectorXd _paramGridLength = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(lengthList.data(), lengthList.size());

    Eigen::VectorXd _ones = Eigen::VectorXd::Ones(lengthList.size());

    _paramGridLengthMat.resize(_totalJoints,_randPointCount);
    _paramGridLengthMat = (_paramGridLength - _ones).rowwise().replicate(_randPointCount);



}

double HLPolarDS::mvnpdf(HLPolarDS::Phi xVal, HLPolarDS::Phi muVal){

    Eigen::MatrixXd sigmaR = Eigen::MatrixXd::Identity(_totalJoints, _totalJoints) * _pdfs.sigma(0);
    Eigen::MatrixXd sigmaC = Eigen::MatrixXd::Identity(_totalJoints, _totalJoints) * _pdfs.sigma(1);

    Eigen::VectorXd diffR = xVal.r - muVal.r;
    Eigen::VectorXd diffC = xVal.c - muVal.c;

    double mvnpdfR = diffR.transpose() * sigmaR.inverse() * diffR;
    double mvnpdfC = diffC.transpose() * sigmaC.inverse() * diffC;

    double mvnpdfVal = std::exp(-0.5*(mvnpdfR + mvnpdfC))/(std::sqrt(std::pow(_pdfs.sigma(0)*_pdfs.sigma(1)*std::pow(2.00*M_PI,2), _totalJoints)));

    return mvnpdfVal;
    
}