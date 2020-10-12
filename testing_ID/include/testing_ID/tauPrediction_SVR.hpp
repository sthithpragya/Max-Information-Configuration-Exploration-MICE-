#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <ctype.h>
#include "libSVMpackage/svm.h"
#include <iostream>
#include <errno.h>
#include <chrono>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

double predictedTau(std::vector<double> jointState, struct svm_model *SVMModel){
    struct svm_node* svmVec;
    svmVec = (struct svm_node *)malloc((jointState.size()+1)*sizeof(struct svm_node));
    
    for (int c=0; c<jointState.size(); c++){
        svmVec[c].index = c+1;  // Index starts from 1; Pre-computed kernel starts from 0
        svmVec[c].value = jointState[c];
        }
    svmVec[jointState.size()].index = -1;   // End of line
    
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double prediction = svm_predict(SVMModel, svmVec);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    
    return prediction;
}