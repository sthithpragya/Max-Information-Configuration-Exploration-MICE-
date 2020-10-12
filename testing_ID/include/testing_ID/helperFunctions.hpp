#define _USE_MATH_DEFINES

#include <algorithm>
#include <vector> 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <functional>

using namespace std;


bool inrange(vector<double> current, vector<double> goal, double bound){

	for(int jointIndex = 0; jointIndex < current.size(); jointIndex++){
		if (!((current[jointIndex] >= goal[jointIndex] - bound) && (current[jointIndex] <= goal[jointIndex] + bound))){
			return false;
		}
	}
	return true;
}


vector<double> sinusoidPos(double _time, vector<double> k, vector<double> ph, int totalJoints, vector<double> qLimLower,vector<double> qLimUpper){

	vector<double> desiredJointPos;
	for(int jointIndex = 0; jointIndex < totalJoints; jointIndex++){		
		double newPos = 0.5*(qLimUpper[jointIndex]-qLimLower[jointIndex])*sin(k[jointIndex]*_time + ph[jointIndex]);

		desiredJointPos.push_back(newPos);
	}
	return desiredJointPos;
}



vector<double> sinusoidVel(double _time, vector<double> k, vector<double> ph, int totalJoints, vector<double> qLimLower, vector<double> qLimUpper, vector<double> qDotLimLower, vector<double> qDotLimUpper){

	vector<double> desiredJointvel;
	for(int jointIndex = 0; jointIndex < totalJoints; jointIndex++){		
		
		double _k = k[jointIndex];
		double _ph = ph[jointIndex];

		double newVel = (qLimUpper[jointIndex]-qLimLower[jointIndex])*0.5*_k*cos(_k*_time+_ph);

		if(newVel > qDotLimUpper[jointIndex]){
			newVel = qDotLimUpper[jointIndex];
		}

		if(newVel < qDotLimLower[jointIndex]){
			newVel = qDotLimLower[jointIndex];
		}

		desiredJointvel.push_back(newVel);
	}

	return desiredJointvel;
}




vector<double> sinusoidAcc(double _time, vector<double> k, vector<double> ph, int totalJoints, vector<double> qLimLower, vector<double> qLimUpper){

	vector<double> desiredJointAcc;
	for(int jointIndex = 0; jointIndex < totalJoints; jointIndex++){		
		double newAcc = -0.5*k[jointIndex]*k[jointIndex]*(qLimUpper[jointIndex]-qLimLower[jointIndex])*0.5*sin(k[jointIndex]*_time + ph[jointIndex]);
		desiredJointAcc.push_back(newAcc);
	}
	return desiredJointAcc;
}






vector<double> sinusoidPos(double _time, vector<double> k, vector<double> ph, vector<double> beta, vector<double> theta, int totalJoints, vector<double> qLimList){

	vector<double> desiredJointPos;
	for(int jointIndex = 0; jointIndex < totalJoints; jointIndex++){		
		double newPos = qLimList[jointIndex]*0.5f*(sin(beta[jointIndex]*_time + theta[jointIndex])+1.0f)*sin(k[jointIndex]*_time + ph[jointIndex]);

		desiredJointPos.push_back(newPos);
	}
	return desiredJointPos;
}
vector<double> sinusoidVel(double _time, vector<double> k, vector<double> ph, vector<double> beta, vector<double> theta, int totalJoints, vector<double> qLimList, vector<double> qDotLimList){

	vector<double> desiredJointvel;
	for(int jointIndex = 0; jointIndex < totalJoints; jointIndex++){		
		
		double _k = k[jointIndex];
		double _theta = theta[jointIndex];
		double _ph = ph[jointIndex];
		double _beta = beta[jointIndex];

		double newVel = qLimList[jointIndex]*0.5f*(
			_k*cos(_k*_time + _ph) + _k*sin(_beta*_time+_theta)*cos(_k*_time+_ph) + _beta*cos(_beta*_time+_theta)*sin(_k*_time+_ph)
		);
		
		if(newVel > qDotLimList[jointIndex]){
			newVel = qDotLimList[jointIndex];
		}

		if(newVel < -qDotLimList[jointIndex]){
			newVel = -qDotLimList[jointIndex];
		}

		desiredJointvel.push_back(newVel);
	}

	return desiredJointvel;
}
