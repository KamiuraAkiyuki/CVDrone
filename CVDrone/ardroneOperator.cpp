//
//  ardroneOperator.cpp
//  CVDrone
//
//  Created by Akiyuki Kamiura on 2016/03/11.
//  Copyright © 2016年 Akiyuki Kamiura. All rights reserved.
//

#include "ardroneOperator.hpp"

bool ARDroneOperator::start(int camera){
    if (!ardrone.open()){
        std::cout << "failed to open ardrone" << std::endl;
        return false;
    }

    ardrone.setCamera(1);
    vx = 0.0; vy = 0.0; vz = 0.0; vr = 0.0;
    
    if (ardrone.onGround()) ardrone.takeoff();
    return true;
}

void ARDroneOperator::moveZ(double meters){
    while (1){
        double now_meters = ardrone.getAltitude();
        if (meters > now_meters){
            ardrone.move3D(0, 0, 0.1, 0);
        } else {
            ardrone.move3D(0, 0, -0.1, 0);
        }
        std::cout << "meters:" << ardrone.getAltitude() << std::endl;
        if (fabs(now_meters - meters) < 0.01) break;
    }
};

void ARDroneOperator::takeoff(){
    ardrone.takeoff();
}

void ARDroneOperator::landing(){
    ardrone.landing();
}

void ARDroneOperator::floating(){
    if (fabs(vx) < 0.01 || fabs(vy) < 0.01){
        vx = 0.05 * (float)rand()/32767.0;
        vy = 0.05 * (float)rand()/32767.0;
    }
    
    cv::Mat cam_img, dst, color_dst;
    cam_img = ardrone.getImage();
    
    Canny(cam_img, dst, 50, 200, 3);
    cvtColor(dst, color_dst, CV_GRAY2BGR);
    
    cv::vector<cv::Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 80, 200, 10);
    
    int line_index = -1;
    double max_length = 0.0;
    for(int i = 0; i < (int)lines.size(); i++){
        line(color_dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
        
        double length = sqrt(pow(abs(lines[i][0] - lines[i][2]), 2) + pow(std::fabs(lines[i][1] - lines[i][3]), 2));
        if (length > max_length) {
            line_index = i;
            max_length = length;
        }
    }
    
    cv::Point pt1, pt2;
    cv::Point center = cv::Point(color_dst.cols/2, color_dst.rows/2);
    if (line_index >= 0){
        pt1 = cv::Point(lines[line_index][0], lines[line_index][1]);
        pt2 = cv::Point(lines[line_index][2], lines[line_index][3]);
        cv::line(color_dst, pt1, pt2, cv::Scalar(255, 0, 0), 3, 8);
        
        cv::Point nearest_point = vertical_point(pt1, pt2, center);
        cv::line(color_dst, nearest_point, center, cv::Scalar(255, 255, 0), 3, 8);
        
        cv::Point direction = nearest_point - center;
        double dirlen = pow(direction.x*direction.x + direction.y*direction.y, 0.5);
        cv::Point direction_unit = cv::Point(direction.x/dirlen, direction.y/dirlen);
        
        vx = -0.05*direction_unit.y;
        vy = -0.05*direction_unit.x;
    }
    
    cv::imshow("cam_img", color_dst);
    
    
};

void ARDroneOperator::track(cv::Point marker, int binalized_cols, int binalized_rows){
    // PIDゲイン
    const double kp = 0.000001;
    const double ki = 0.000;
    const double kd = 0.000;
    
    // 誤差 (画像座標系-機体座標系の変換のため符号とXYが逆)
    double error_x = (binalized_rows / 2 - marker.y);
    double error_y = (binalized_cols / 2 - marker.x);
    
    
    // 時間 [s]
    static int64 last_t = 0.0;
    double dt = (cv::getTickCount() - last_t) / cv::getTickFrequency();
    last_t = cv::getTickCount();
    
    // 積分項
    static double integral_x = 0.0, integral_y = 0.0;
    if (dt > 0.1) {
        // リセット
        integral_x = 0.0;
        integral_y = 0.0;
    }
    integral_x += error_x * dt;
    integral_y += error_y * dt;
    
    // 微分項
    static double previous_error_x = 0.0, previous_error_y = 0.0;
    if (dt > 0.1) {
        // リセット
        previous_error_x = 0.0;
        previous_error_y = 0.0;
    }
    double derivative_x = (error_x - previous_error_x) / dt;
    double derivative_y = (error_y - previous_error_y) / dt;
    previous_error_x = error_x;
    previous_error_y = error_y;
    
    // 操作量
    vx = kp * error_x + ki * integral_x + kd * derivative_x;
    vy = kp * error_y + ki * integral_y + kd * derivative_y;
    vz = 0.0;
    vr = 0.0;
    std::cout << "(vx, vy)" << "(" << vx << "," << vy << ")" << std::endl;
}

void ARDroneOperator::quit(){
    ardrone.close();
}

cv::Mat ARDroneOperator::image(){
    return ardrone.getImage();
}

void ARDroneOperator::move(){
    ardrone.move3D(vx, vy, vz, vr);
}

cv::Point ARDroneOperator::vertical_point(cv::Point A, cv::Point B, cv::Point P){
    cv::Point AB,AP;//ベクトルAB AP
    
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    AP.x = P.x - A.x;
    AP.y = P.y - A.y;
    
    //ABの単位ベクトルを計算
    double len = pow(AB.x* AB.x + AB.y*AB.y, 0.5);
    cv::Point nAB = cv::Point(AB.x/len, AB.y/len);
    
    //Aから線上最近点までの距離（ABベクトルの後ろにあるときはマイナス値）
    double dist_AX = nAB.x*AP.x + nAB.y*AP.y;
    
    //線上最近点
    cv::Point ret;
    ret.x = A.x + ( nAB.x * dist_AX );
    ret.y = A.y + ( nAB.y * dist_AX );
    
    return ret;
}


