//
//  main.cpp
//  OpenCVTest
//
//  Created by Akiyuki Kamiura on 2016/03/04.
//  Copyright © 2016年 Akiyuki Kamiura. All rights reserved.
//

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "figureFinder.hpp"
#include "ardrone/ardrone.h"
#include "ardroneOperator.hpp"

int main (int argc, char **argv){
    
    int landing_point = 0;
    int colmode = 0;
    
    ARDroneOperator op;
    if (!op.start(1)) return -1;
    
    op.moveZ(1.0);
    
    while (1){
        cv::Mat img = op.image();

        FigureFinder figureFinder = FigureFinder();
        figureFinder.setImage(img);
        figureFinder.setColorMode(colmode);
        figureFinder.findCircle();
        img = figureFinder.getImage();
        cv::Mat binalized = figureFinder.getMaskedImage();
        cv::Point marker = figureFinder.getMarker();
        
        static int track = 0;
        if (marker.x != -1 || marker.y != -1) track = 1;
        
        if (track){
            op.track(marker, binalized.cols, binalized.rows);
        } else {
            op.floating();
        }
        op.move();
        
        // 着地判定
        double distanceToMarker = sqrt(pow(abs(marker.x - img.cols/2), 2) + pow(abs(marker.y - img.rows/2), 2));
        if (distanceToMarker < (30 - landing_point*5)){
            landing_point++;
            if (landing_point <= 5){
                cv::putText(img, "landing_p:", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA);
                op.moveZ(1.0 - (float)landing_point * 0.1);
            } else {
                cv::putText(img, "ok", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA);
                landing_point = 0;
                colmode++;
                op.landing();
            }
        }
        
        // 映像出力
        if (colmode == 0){
            cv::putText(img, "searching red marker", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        } else if (colmode == 1){
            cv::putText(img, "searching green marker", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        } else if (colmode == 2){
            cv::putText(img, "searching blue marker", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1, CV_AA);
        } else if (colmode == 3){
            cv::putText(img, "END", cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1, CV_AA);
            cv::imshow("camera", img);
            break;
        }
        cv::imshow("camera", img);
        
        // key入力
        int key = cv::waitKey(1);
        if (key == 'q') break;
    }
    
    op.quit();
    
    return 0;
}
