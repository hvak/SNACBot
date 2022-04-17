#!/usr/bin/env python3

import os
import cv2
import torch
import numpy as np

class Object_Detector:
    def __init__(self, device):
        print("SnackNet Created")
        self.device = device
        path = os.getcwd() + '/yolov5'
        self.model = torch.hub.load(path, 'custom', 'snacnetV2.pt', source='local').eval().to(device)
        print("Finished loading SnackNet")
        self.currentObject = []
        
    def callback(self, image):
        output = self.model.forward(image)
        if len(output.xyxy) > 0 and len(output.xyxy[0]) > 0:
            index = torch.argmax(output.xyxy[0], dim=0)[4]  
            chosen_object = output.xyxy[0][index]
            if chosen_object[4] > 0.85:
                x1 = int(chosen_object[0])
                y1 = int(chosen_object[1])
                x2 = int(chosen_object[2])
                y2 = int(chosen_object[3])
                
                nrows, ncols = image.shape[0], image.shape[1]
                bool_array = np.full((nrows, ncols), True)
                bool_array[y1:y2, x1:x2] = False
                
                converted_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                grad = cv2.Canny(converted_image, 100, 200)
                grad[bool_array] = 0
                grad = cv2.medianBlur(grad.astype(np.uint8), 3)
                
                cv2.imwrite("gradient_photo.jpg", grad)             
                cv2.imwrite("raw_photo.jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                self.currentObject = chosen_object.tolist()
                print(self.currentObject)


def main():
    torch.cuda.empty_cache()
    device = None
    if torch.cuda.is_available():
        device = torch.device('cuda:0')
        print("Using GPU!")
    else:
        device = torch.device('cpu')
        print("Warning: Could not find GPU")
    od = Object_Detector(device)

if __name__ == '__main__':
    main()
