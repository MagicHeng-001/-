import os
import pickle as pk
import pandas as pd

# 从原始数据中筛选中阈值大于100的类别送进gan
# 各类别数量
# 3    1565
# 5     962
# 2     714
# 0     490
# 1     441
# 6      28
# 8       7
# 7       4
# 4       2

#gan数据label的映射    TrueLable2GanLable:  {0: 0, 1: 1, 2: 2, 3: 3, 5: 4}

sessionLen = 500
dataDir = r'../trainTestData/500_augmentTrain.pkl'
traditionDataDir = r'../trainData/BorderlineSMOTE_new.pkl'
#不需要扩充的数据集
trainDir = r'../trainTestData/500_train.pkl'
ganDataSavePath = r'./tempTrainData/'+ str(sessionLen) + 'ganData.pkl'
tempDataSavePath = r'./tempTrainData/'+ str(sessionLen) + 'tempTrainData.pkl'
threadshold = 100


f = open(dataDir,'rb')
orginData = pk.load(f)
noAugData = pk.load(open(trainDir,'rb'))
traditionData = pk.load(open(traditionDataDir,'rb'))
class2Num = dict(orginData.loc[:,'label'].value_counts())
ganClass = []
trainData = pd.DataFrame()
trainData = trainData.append(noAugData)
ganData = pd.DataFrame()
for key,value in class2Num.items():
    if value > threadshold:
        # 该类别的数据用Gan生成
        ganClass.append(key)
        ganData = ganData.append(orginData[orginData['label'].isin([key])])
    else:
        #用经典算法生成的数据
        trainData = trainData.append(traditionData[traditionData['label'].isin([key])])

#需要对Gan数据的标签从0开始重新标号
TrueLable2GanLable = {}
ganLable = sorted(ganData['label'].unique().tolist())
count = 0
for i in ganLable:
    TrueLable2GanLable[i] = count
    count+=1
print('TrueLable2GanLable: ',TrueLable2GanLable)
#修改label值
def lable(df):
    return TrueLable2GanLable[df['label']]
ganData['label'] = ganData.apply(lambda row : lable(row),axis=1)

with open(ganDataSavePath,'wb') as ganSaveF:
    with open(tempDataSavePath,'wb') as tempF:
        pk.dump(ganData,ganSaveF)
        pk.dump(trainData,tempF)
print(ganClass)