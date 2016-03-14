
//
//  hough_circle.cpp
//  OpenCVTest
//
//  Created by Akiyuki Kamiura on 2016/03/06.
//  Copyright © 2016年 Akiyuki Kamiura. All rights reserved.
//

#include "figureFinder.hpp"

void FigureFinder::findCircle(){
    
    if (colorMode == FIND_RED){
        masked_img = colorExtraction(mat_img, 160, 180, 100, 255, 0, 255);
    } else if (colorMode == FIND_GREEN){
        masked_img = colorExtraction(mat_img, 50, 65, 50, 255, 30, 255);
    } else if (colorMode == FIND_BLUE){
        masked_img = colorExtraction(mat_img, 110, 120, 100, 255, 100, 255);
    }
    
    cvtColor(masked_img, masked_img, CV_BGR2GRAY);
    GaussianBlur(masked_img, masked_img, cv::Size(9, 9), 2, 2);
    
    cv::vector<cv::Vec3f> circles;
    HoughCircles(masked_img, circles, CV_HOUGH_GRADIENT, 2, 50, 200, 30, 10, 150);
    
    for(size_t i = 0; i < circles.size(); i++){
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(mat_img, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
        circle(mat_img, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
    }
    
    int circle_index = -1;
    double max_area = 0.0;
    for (int i = 0; i < (int)circles.size(); i++) {
        double area = fabs(pow(cvRound(circles[i][2]), 2)*M_PI);
        if (area > max_area) {
            circle_index = i;
            max_area = area;
        }
    }
    
    trackMarker.x = -1;
    trackMarker.y = -1;
    
    if (circle_index >= 0){
        trackMarker.x = cvRound(circles[circle_index][0]);
        trackMarker.y = cvRound(circles[circle_index][1]);
        int radius = cvRound(circles[circle_index][2]);
        
        cv::rectangle(mat_img, cv::Point(trackMarker.x - radius, trackMarker.y - radius),
                      cv::Point(trackMarker.x + radius, trackMarker.y + radius), cv::Scalar(0, 255, 255));
    }
};

void FigureFinder::findTriangle(){

};

void FigureFinder::findSquare(){

};

void FigureFinder::setImage(cv::Mat img){
    mat_img = img;
};

cv::Mat FigureFinder::getImage(){
    return mat_img;
};

cv::Mat FigureFinder::getMaskedImage(){
    return masked_img;
};

void FigureFinder::setColorMode(int color){
    if (color >= COLOR_END) std::cout << "out of limit: colormode not changed" << std::endl;
    colorMode = static_cast<colorKind>(color);
};

cv::Mat FigureFinder::colorExtraction(cv::Mat src, int ch1Lower, int ch1Upper, int ch2Lower, int ch2Upper, int ch3Lower, int ch3Upper){
    cv::Mat colorImage;
    cv::Mat dst;
    int lower[3];
    int upper[3];
    
    cv::Mat lut = cv::Mat(256, 1, CV_8UC3);
    
    cv::cvtColor(src, colorImage, CV_BGR2HSV);
    
    lower[0] = ch1Lower;
    lower[1] = ch2Lower;
    lower[2] = ch3Lower;
    
    upper[0] = ch1Upper;
    upper[1] = ch2Upper;
    upper[2] = ch3Upper;
    
    for (int i = 0; i < 256; i++){
        for (int k = 0; k < 3; k++){
            if (lower[k] <= upper[k]){
                if ((lower[k] <= i) && (i <= upper[k])){
                    lut.data[i*lut.step+k] = 255;
                }else{
                    lut.data[i*lut.step+k] = 0;
                }
            }else{
                if ((i <= upper[k]) || (lower[k] <= i)){
                    lut.data[i*lut.step+k] = 255;
                }else{
                    lut.data[i*lut.step+k] = 0;
                }
            }
        }
    }
    
    //LUTを使用して二値化
    cv::LUT(colorImage, lut, colorImage);
    
    //Channel毎に分解
    std::vector<cv::Mat> planes;
    cv::split(colorImage, planes);
    
    //マスクを作成
    cv::Mat maskImage;
    cv::bitwise_and(planes[0], planes[1], maskImage);
    cv::bitwise_and(maskImage, planes[2], maskImage);
    
    //出力
    cv::Mat maskedImage;
    src.copyTo(maskedImage, maskImage);
    return maskedImage;
};

cv::Point FigureFinder::getMarker(){
    return trackMarker;
};





