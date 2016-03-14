//
//  ardroneOperator.hpp
//  CVDrone
//
//  Created by Akiyuki Kamiura on 2016/03/11.
//  Copyright © 2016年 Akiyuki Kamiura. All rights reserved.
//

#ifndef ardroneOperator_hpp
#define ardroneOperator_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ardrone/ardrone.h"
#include "figureFinder.hpp"

class ARDroneOperator {
public:
    bool start(int camera);
    void takeoff();
    void landing();
    void moveZ(double meters);
    cv::Mat image();
    void track(cv::Point marker,
               int binalized_cols, int binalized_rows);
    void floating();
    void move();
    void quit();
    
private:
    ARDrone ardrone;
    double vx, vy, vz, vr;
    cv::Point vertical_point(cv::Point a,
                             cv::Point b, cv::Point p);
};

#endif /* ardroneOperator_hpp ;*/
