import csv
import os

repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]
CSV_HEADER=["q_des"]+['q0','q1','q2','q3','q4','q5']+["T_des"]+repeat_string+["T_target"]+repeat_string+\
    ["worldFrame"]+repeat_string+["ECM_T_PSM_RCM"]+repeat_string+["psm3_T_cam"]+repeat_string+["offset"]+\
    ['x','y','z']

ROOT_PATH='optimizationData'

class OptimizationDataLogger:
    def __init__(self):
        print("Init Optimization Datalogger")
        self.record_filename=None
        self.file_count=1

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
        with open(self.record_filename_pc1,'w',newline='') as file_object:
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
        offset
        '''
        row_to_write=[] #Inits the row that we will write data to

        for i in range(len(data_list)):
            row_to_write.extend("")
            row_to_write.extend(data_list[i].tolist())
        
        with open(self.record_filename,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)
            file_object.close()
        

    

    
