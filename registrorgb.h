/**
 * File: registrorgb.cpp
 * Date: March 2013
 * Author: Giacomo Picchiarelli <gpicchiarelli@gmail.com>
 * Description: test NARF - SURF features (pcl library, opencv)
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

#include "TwoWayMatcher.h"

using namespace cv;
using namespace std;

class RegistroRGB
{
public:
    typedef vector<float> p_desc;
    typedef vector<cv::KeyPoint> p_keyp;

    RegistroRGB(int size);
    void addFrame(string name, p_desc descriptor, p_keyp keypoints);
    int inliersRGB(int src,int dst);
    Mat getImageAt(int pos);

private:
    vector<string> _name;
    vector<p_desc> _descriptors;
    vector<p_keyp> _keypoints;
};

#endif // REGISTRORGB_H
