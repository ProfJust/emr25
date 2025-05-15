# SRO_openCV_sw02_saveImg2File.py
# .......................................
# Python program to explain cv2.imwrite() method
# importing cv2 
import cv2

# importing os module  
import os

# Image path  
image_path = r'C:\Users\RoboAdmin\mySciebo\_EMR25\emr25\openCV\bild_von_webcam.png'

# Image directory
directory = r'C:\Users\RoboAdmin\mySciebo\_EMR25\emr25\openCV'

# Using cv2.imread() method
# to read the image
img = cv2.imread(image_path)

# Change the current directory 
# to specified directory 
os.chdir(directory)

# List files and directories  
print("list directory before saving image:")  
print(os.listdir(directory))  

# Filename
filename = 'savedImage.jpg'

# Using cv2.imwrite() method
# Saving the image
cv2.imwrite(filename, img)

# List files and directories  
print("After saving image:")  
print(os.listdir(directory))

print('Successfully saved')
