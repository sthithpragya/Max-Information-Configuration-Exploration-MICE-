#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <iterator>
#include "yaml-cpp/yaml.h"

using namespace std;
 
int main()
{   
    YAML::Node paramLoaded = YAML::LoadFile("./../param.yaml");

    int totalJoints = paramLoaded["totalJoints"].as<int>();
    string savePath = paramLoaded["savePath"].as<string>();
    bool learnErrorModel = paramLoaded["learnErrorModel"].as<bool>();
    
    int startModelIndex = paramLoaded["startModelIndex"].as<int>();
    int endModelIndex = paramLoaded["endModelIndex"].as<int>();

    string testDataFileName;
    string dataName;
    string resultName;

    if(learnErrorModel){
        testDataFileName = savePath + "/sparseFormat/sparseErrorTestData";
        dataName = "sparseErrorTrainData";
        resultName = "PredError";
    }
    else{
        testDataFileName = savePath + "/sparseFormat/sparseTestData";
        dataName = "sparseTrainData";
        resultName = "PredTau";
    }

    string dataExtension = ".dat";
    string modelExtension = ".dat.model";

    int process = paramLoaded["process"].as<int>(); 
    int kernel = paramLoaded["kernel"].as<int>();
    double epsilon = paramLoaded["epsilon"].as<double>();
    double nu = paramLoaded["nu"].as<double>();

    string testAccFileName = "testAcc.txt";
    std::ofstream outfile;

    for(int modelIndex = startModelIndex; modelIndex < endModelIndex; modelIndex = modelIndex + 1){

        outfile.open(testAccFileName, std::ios_base::app); // append instead of overwrite
        outfile << "---------------------";
        outfile << "\n";

        string currenTestDataFile = testDataFileName + to_string(modelIndex) + dataExtension;
        string currentTrainedModelFile = dataName + to_string(modelIndex) + modelExtension;
        string currentResultFileName = savePath + "/SVM_Regression_Results/model" + to_string(modelIndex) + "Test" + resultName + ".csv";

        char testCommand[75];
        sprintf (testCommand, "./thundersvmPackage/build/bin/thundersvm-predict %s %s %s | tee -a %s", currenTestDataFile.c_str(), currentTrainedModelFile.c_str(), currentResultFileName.c_str(), testAccFileName.c_str());
        system(testCommand);
        outfile.close();
    }
}