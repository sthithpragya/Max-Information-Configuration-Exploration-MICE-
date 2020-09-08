#!/usr/bin/env python
import os.path
import numpy as np
import torch
import torch.nn.functional as F
import tools
import itertools
import custom_models
import yaml

with open('./../param.yaml',"r") as learningStream:
	paramLoaded = yaml.safe_load(learningStream)

savePath = paramLoaded["savePath"]
totalJoints = paramLoaded["totalJoints"]
learnErrorModel = paramLoaded["learnErrorModel"]

eta = paramLoaded["etaCoupled"]
nb_hidden_layers = paramLoaded["layersCoupled"]
nb_hidden_neurons = paramLoaded["neuronsCoupled"]

if learnErrorModel:
    trainDataFileName = "normalisedErrorTrainData"
    testDataFileName = "normalisedErrorTestData"
else:
    trainDataFileName = "normalisedTrainData"
    testDataFileName = "normalisedTestData"

trainDataFileName = os.path.join(savePath, trainDataFileName + ".csv")
testDataFileName = os.path.join(savePath, testDataFileName + ".csv")

trainData = np.genfromtxt(trainDataFileName, dtype=float, delimiter=',') # np array of 21 columns (q, qdot, qddot)
testData = np.genfromtxt(testDataFileName, dtype=float, delimiter=',') # np array of 7 columns with each column corresponding to that joint's torque

np.random.shuffle(trainData)

train_data = torch.from_numpy(trainData)
test_data = torch.from_numpy(testData)

if torch.cuda.is_available():
    device = torch.device('cuda')
else:
    device = torch.device('cpu')          

train_data = train_data.float()
test_data = test_data.float()

train_data = train_data.to(device)
test_data = test_data.to(device)

#-----------------------------

train_input = train_data[:, 0:3*totalJoints]
train_target = train_data[:, 3*totalJoints:]

test_input = test_data[:, 0:3*totalJoints]
test_target = test_data[:, 3*totalJoints:]
                    
debug = True

# List of seed to use for randomization
s = 0
f = 0
seeds = [0]
nb_epochs = 400
batch_size = 10000
nb_folds = 1

# Train network
model = custom_models.InverseDynamicModel(nb_hidden_layers, nb_hidden_neurons).to(device)

print("training the model")

tools.train_model(model, train_input, train_target, nb_epochs, 
            batch_size, eta, s, len(seeds), f, nb_folds, 
            debug)

# Saving
if learnErrorModel:
    torch.save(model, 'errorModel.pt')    
else:
    torch.save(model, 'torqueModel.pt')

train_error = tools.compute_loss(model, train_input, train_target)
test_error = tools.compute_loss(model, test_input, test_target)

result_train = torch.empty(len(seeds), nb_folds).to(device)
result_test = torch.empty(len(seeds), nb_folds).to(device)

result_train[s, f] = train_error
result_test[s, f] = test_error


results_grid = []
results_grid = results_grid + [result_train.mean()]
results_grid = results_grid + [result_test.mean()]

print(results_grid)    

# Save the training accuracy
header = "train_mse,test_mse"

if learnErrorModel:
    np.savetxt('results_errorModel.csv',[np.array(results_grid)], delimiter=',',header=header,comments='')
else:
    np.savetxt('results_torqueModel.csv',[np.array(results_grid)], delimiter=',',header=header,comments='')