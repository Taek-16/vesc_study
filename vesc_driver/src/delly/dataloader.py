import torch
import os
import pickle
import cv2

class DellyDataset(torch.utils.data.Dataset):
    def __init__(self, condition_label_make = False, transform = None):
        self.x = list()
        self.y = list()
        self.condition = list()
        self.transform = transform
        self.make()
        for k in self.dict:
            v = self.dict[k]
            if v[0]!=0:
                self.x.append(v[-1])
                self.y.append(v[0])
        if condition_label_make:
            self.do_condition_labeling()
            with open('./condition/condition.pickle', 'wb') as f:
                pickle.dump(self.condition, f, pickle.HIGHEST_PROTOCOL)
        else:
            with open("./condition/condition.pickle", 'rb') as f:
                condition = pickle.load(f)
                self.condition = condition



    def axis_distribution(self):
        import matplotlib.pyplot as plt
        plt.hist(self.y, bins=50)
        plt.show()

    def make(self):
        routes = os.listdir("./dataset")
        self.dict = dict()
        not_matched = 0
        not_matched_list = list()
        for route in routes:
            print(" + processing route :: ",route)
            trials = os.listdir("./dataset/"+route)
            for trial in trials:
                full_path = "./dataset/" + route +"/" + trial
                files = os.listdir(full_path)
                for file in files:
                    if('pickle' in file):
                        #print("./dataset/" + route +"/" + trial + "/"+file)
                        try:
                            with open("./dataset/" + route +"/" + trial + "/"+file, 'rb') as f:
                                data = pickle.load(f)
                        except:
                            break
                        timestamp = data['time_stamp']
                        axis = data['servo_position'] # 0~1
                        for t in range(len(timestamp)-1):
                            if(timestamp[t] != timestamp[t+1]):
                                self.dict[timestamp[t]] = [axis[t]]
                    elif("rgb" in file):
                        full_img_path = "./dataset/" + route +"/" + trial + "/"+file
                        timestamp = file[:-8]
                        try:
                            self.dict[timestamp].append(full_img_path)
                        except:
                            not_matched += 1

                    elif("depth" in file):
                        pass

        for k in self.dict:
            v = self.dict[k]
            if len(v)==1:
                not_matched_list.append(k)
        for notm in not_matched_list:
            #print(self.dict[notm])
            del self.dict[notm]
        print(" dataset length :: ", len(self.dict))
        print(" not matched (img, label) : ",not_matched)

    def do_condition_labeling(self):
        import matplotlib.pyplot as plt
        # make condition of the image
        for idx in range(len(self.x)):
            x = cv2.imread(self.x[idx], cv2.IMREAD_COLOR)
            cv2.imshow('img',x)
            command = cv2.waitKey(0)
            if command == ord('w'):
                # go straight
                self.condition.append(0)
            elif command == ord('a'):
                # turn left
                self.condition.append(1)
            elif command == ord('d'):
                # turn left
                self.condition.append(2)


    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        x = cv2.imread(self.x[idx], cv2.IMREAD_COLOR)
        y = self.y[idx]
        cond = self.condition[idx]
        if self.transform is not None:
            x = self.transform(image=x)['image']
        return x, y, cond

if __name__=="__main__":
    dataset = DellyDataset()
    dataset.axis_distribution()

    if True:
        for i in range(len(dataset)):
            img, target, cond = dataset[i]
            if cond ==0:
                cv2.arrowedLine(img, (100, 150), (100, 50), (0, 0, 255), 3, 8, 0, 0.1)
            elif cond ==1:
                cv2.arrowedLine(img, (150, 50), (50, 50), (0, 0, 255), 3, 8, 0, 0.1)
            elif cond ==2:
                cv2.arrowedLine(img, (50, 50), (150, 50), (0, 0, 255), 3, 8, 0, 0.1)
            cv2.imshow('img', img)
            cv2.waitKey(0)
            print(target, cond, img.shape)
