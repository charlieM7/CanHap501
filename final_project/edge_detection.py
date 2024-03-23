# Code adapted from LearnOpenCV
# https://learnopencv.com/edge-detection-using-opencv/

import cv2
import argparse
import pathlib


def main(args):
    img = cv2.imread(str(args.image)) 
   
    # Data preprocess
    grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(grayscale_img, (3,3), 0) 

    # Canny Edge Detection
    edges = cv2.Canny(image=blur_img, threshold1=100, threshold2=200) # Canny Edge Detection
    cv2.imshow('Canny Edge Detection', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Sobel Edge Detection
    sobel_xy = cv2.Sobel(src=blur_img, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
    cv2.imshow('Sobel XY', sobel_xy)
    cv2.waitKey(0)
    

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=pathlib.Path, default='./depth_images/depth_image.png')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    main(args)