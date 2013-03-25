/**
 * File: democ.cpp
 * Date: November 2012
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

#include "registrorgb.h"

RegistroRGB::RegistroRGB(int size)
{
    this->_name.reserve(size);
    this->_descriptors.reserve(size);
    this->_keypoints.reserve(size);
}
void RegistroRGB::addFrame(string name, p_desc descriptor, p_keyp keypoints){
    this->_name.push_back(name);
    this->_descriptors.push_back(descriptor);
    this->_keypoints.push_back(keypoints);
}
int RegistroRGB::inliersRGB(int src,int dst)
{
    int numInliers = 0;

    cv::Mat descriptors1;
    cv::Mat descriptors2;
    vector<cv::KeyPoint> keypoints1;
    vector<cv::KeyPoint> keypoints2;

    descriptors1 = Mat(this->_descriptors[src]);
    descriptors2 = Mat(this->_descriptors[dst]);

    keypoints1 = this->_keypoints[src];
    keypoints2 = this->_keypoints[dst];

    TwoWayMatcher matcher(TWM_FLANN);
    vector<DMatch> matches;
    Mat mask1, mask2;
    matcher.match(descriptors1, descriptors2, matches, mask1, mask2);

    // Initializes points lists
    vector<cv::Point2f > p1vec(matches.size());
    vector<cv::Point2f > p2vec(matches.size());
    Mat points1(p1vec);
    Mat points2(p2vec);

    vector<uchar> inliersMask;

    findHomography(Mat(points1), Mat(points2), inliersMask, CV_FM_RANSAC, 1);
    numInliers =  count(inliersMask.begin(), inliersMask.end(), 1);

    inliersMask.clear();
    p1vec.clear();
    p2vec.clear();

    return numInliers;
}
