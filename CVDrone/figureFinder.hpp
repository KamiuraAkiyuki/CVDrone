//
//  figureFinder.hpp
//  OpenCVTest
//
//  Created by Akiyuki Kamiura on 2016/03/06.
//  Copyright © 2016年 Akiyuki Kamiura. All rights reserved.
//

#ifndef figureFinder_hpp
#define figureFinder_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

typedef enum {
    FIND_RED,
    FIND_GREEN,
    FIND_BLUE,
    GRAY_SCALE,
    COLOR_END
} colorKind;

class FigureFinder {
public:
    FigureFinder(){};
    void setImage(cv::Mat img);
    cv::Mat getImage();
    cv::Mat getMaskedImage();
    cv::Point getMarker();
    void findCircle();
    void findTriangle();
    void findSquare();
    void setColorMode(int color);
    
private:
    cv::Mat mat_img;
    cv::Mat masked_img;
    colorKind colorMode;
    cv::Point trackMarker;
    cv::Mat colorExtraction(cv::Mat src,
                            int ch1Lower, int ch1Upper,
                            int ch2Lower, int ch2Upper,
                            int ch3Lower, int ch3Upper);
};

#endif /* figureFinder_hpp */
