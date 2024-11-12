import csv
import os
from datetime import datetime
import numpy as np
import pandas as pd

repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]
CSV_HEADER=["System Time","q_des"]+['q0','q1','q2','q3','q4','q5']+["T_des"]+repeat_string+["T_target"]+repeat_string+\
    ["worldFrame"]+repeat_string+["ECM_T_PSM_RCM"]+repeat_string+["psm3_T_cam"]+repeat_string+["offset"]+\
    ['x','y','z','IK Trigered','ECM_T_PSM3']+repeat_string

ROOT_PATH='optimizationData'

class OptimizationDataLogger:
    def __init__(self):
        print("Init Optimization Datalogger")
        self.record_filename=None
        self.file_count=1
        self.read_row_count=0

    def initRecording(self):
        #Check if path exists
        if not os.path.exists(ROOT_PATH):
            os.makedirs(ROOT_PATH)
        #Initialize a new CSV file to save PC1 data to
        file_name=ROOT_PATH+'/Data_'+str(self.file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                self.file_count+=1
                file_name=ROOT_PATH+'/Data_'+str(self.file_count)+'.csv'             
            else:
                break
        self.record_filename=file_name

        #Inits Header
        with open(self.record_filename,'w',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(CSV_HEADER)
            file_object.close()
    def writeRow(self,data_list):
        '''
        data_list contains a list where each index is:, in a list format
        q_des, 
        T_des, 
        T_target, 
        worldFrame,
        ECM_T_PSM_RCM, 
        psm3_T_cam,
        offset,
        IK Triggered Boolean,
        ECM_T_PSM3

        '''
        #Gets system time
        curr_time=datetime.now()
        curr_time=curr_time.strftime("%H:%M:%S.%f")

        row_to_write=[] #Inits the row that we will write data to
        row_to_write.append(curr_time)
        row_to_write.append("")

        #6 joint params
        row_to_write.extend(data_list[0])
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[1]))
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[2]))
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[3]))
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[4]))
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[5]))


        #Adds the offset
        row_to_write.append("")
        row_to_write.extend(data_list[6].tolist())

        #Adds the boolean flag
        row_to_write.append(data_list[7])
        row_to_write.append("")
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[8]))

        
        
        with open(self.record_filename,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)
            file_object.close()

    def initReading(self,filname=None):
        #Filename is pre-defined filename, if None use highest numbered filename
        if filname is not None:
            self.record_filename=ROOT_PATH+'/'+filname+'.csv'
        else:
            file_count=1
            file_name=ROOT_PATH+'/Data_'+str(file_count)+'.csv'
            while True:
                if os.path.isfile(file_name):
                    file_count+=1
                    file_name=ROOT_PATH+'/Data_'+str(file_count)+'.csv'
                else:
                    break
            self.record_filename=ROOT_PATH+'/Data_'+str(file_count-1)+'.csv'
        #Initializes the counter to index the rows:
        self.read_row_count=0

    def readDataRow(self):
        self.read_row_count+=1
        data_row=pd.read_csv(self.record_filename,skiprows=self.read_row_count)
        data_list=data_row.iloc[0].to_list()

        #Returns the data as a list such that:
        '''
        q_des, 
        T_des, 
        T_target, 
        worldFrame,
        ECM_T_PSM_RCM, 
        psm3_T_cam,
        offset
        '''
        system_time=data_list[0]
        q_des=data_list[2:8]
        T_des=data_list[9:21]
        T_target=data_list[22:34]
        worldFrame=data_list[35:47]
        ECM_T_PSM_RCM=data_list[48:60]
        psm3_T_cam=data_list[61:73]
        offset=data_list[74:77]
        IK_Triggered=data_list[77]
        ECM_T_PSM3=data_list[78:90]


        q_des=np.array(q_des,dtype=np.float64)
        T_des=self.ConvertDataRow_ToNPFrame(T_des)
        T_target=self.ConvertDataRow_ToNPFrame(T_target)
        worldFrame=self.ConvertDataRow_ToNPFrame(worldFrame)
        ECM_T_PSM_RCM=self.ConvertDataRow_ToNPFrame(ECM_T_PSM_RCM)
        psm3_T_cam=self.ConvertDataRow_ToNPFrame(psm3_T_cam)
        offset=np.array(offset,dtype=np.float64)   
        ECM_T_PSM3=self.ConvertDataRow_ToNPFrame(ECM_T_PSM3)

        return system_time,q_des,T_des,T_target,worldFrame,ECM_T_PSM_RCM,psm3_T_cam,offset,IK_Triggered,ECM_T_PSM3





    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[transform[0,3],transform[1,3],transform[2,3],\
                    transform[0,0],transform[0,1],transform[0,2],\
                    transform[1,0],transform[1,1],transform[1,2],\
                    transform[2,0],transform[2,1],transform[2,2]]
        
        
        return string_list
    

    def ConvertDataRow_ToNPFrame(self,data_list):
        transform=np.array([data_list[3],data_list[4],data_list[5],data_list[0]],
                           [data_list[6],data_list[7],data_list[8],data_list[1]],
                           [data_list[9],data_list[10],data_list[11],data_list[2]],
                           [0,0,0,1],dtype=np.float64)
        return transform
        

    

    
