from kukaDHfunctions import *
import yaml
import io

with open(r'param.yaml') as stream:
    paramLoaded = yaml.safe_load(stream)

savePath = paramLoaded["savePath"]

readFileName = "recordedDesiredJointData"
readFileName = os.path.join(savePath, readFileName + ".csv")

writeFileName = "computedDesiredTaskData"
writeFileName = os.path.join(savePath, writeFileName + ".csv")

dataWrite = open(writeFileName, "w")
csvwriter = csv.writer(dataWrite)

with open(readFileName) as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        floatRow = map(float,row)
        tempTransformation = kukaDH(floatRow)
        tempTranslation = tempTransformation[0:3,3]
        tempTranslation = tempTranslation.transpose()
        tempTranslation[0] = -tempTranslation[0]
        tempTranslation[1] = -tempTranslation[1]

        tempTranslation = tempTranslation.tolist()
        csvwriter.writerow(map(str, tempTranslation))












        