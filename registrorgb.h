/**
 * File: democ.cpp
 * Date: November 2012
 * Author: Giacomo Picchiarelli <gpicchiarelli@gmail.com>
 * Description: test NARF - SIRF features (pcl library, opencv)
 *
 * This file is licensed under a Creative Commons
 * Attribution-NonCommercial-ShareAlike 3.0 license.
 * This file can be freely used and users can use, download and edit this file
 * provided that credit is attributed to the original author. No users are
 * permitted to use this file for commercial purposes unless explicit permission
 * is given by the original author. Derivative works must be licensed using the
 * same or similar license.
 * Check http://creativecommons.org/licenses/by-nc-sa/3.0/ to obtain further
 * details.
 *
 */

#ifndef REGISTRORGB_H
#define REGISTRORGB_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>
#include <string.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;
using namespace std;

class CoppiaRGB
{
public:
    CoppiaRGB(string filename,vector<float> descriptors, vector<cv::KeyPoint> keypoints);
    string name;
    vector<float> descriptors;
    vector<cv::KeyPoint> keypoints;

private:

};

#endif // REGISTRORGB_H
