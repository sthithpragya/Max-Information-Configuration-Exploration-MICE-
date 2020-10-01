from kukaDHfunctions import *
import yaml
import io

with open(r'param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

savePath = paramLoaded["savePath"]

readFileName = "recordedActualJointData"
readFileName = os.path.join(savePath, readFileName + ".csv")

writeFileName = "computedActualTaskData"
writeFileName = os.path.join(savePath, writeFileName + ".csv")

dataWrite = open(writeFileName, "w")
csvwriter = csv.writer(dataWrite)

print("CONVERTING RECORDED JOINT-SPACE DATA TO TASK-SPACE DATA")

with open(readFileName) as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        floatRow = map(float,row)
        tempTransformation = kukaDH(floatRow[0:7])
        tempTranslation = tempTransformation[0:3,3]
        tempTranslation = tempTranslation.transpose()
        tempTranslation[0] = -tempTranslation[0]
        tempTranslation[1] = -tempTranslation[1]

        tempTranslation = tempTranslation.tolist()
        csvwriter.writerow(map(str, tempTranslation))












        