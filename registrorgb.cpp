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

RegistroRGB::RegistroRGB(string directory)
{
    this->listFile(directory,&this->__files_list_rgb);
}


cv::Mat RegistroRGB::getImageAt(int i){
    return cv::imread(this->__files_list_rgb[i]);   
}

int RegistroRGB::inliersRGB(int src,int dst)
{
    int numInliers = 0;
    
    cv::SURF surf(300, 5, 4, true);
   
    cv::Mat image1 = cv::imread(this->__files_list_rgb[src]);
    cv::Mat msk1;
    vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    surf(image1, msk1, keypoints1, descriptors1);
    
    cv::Mat image2 = cv::imread(this->__files_list_rgb[dst]);
    cv::Mat msk2;
    vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    surf(image2, msk2, keypoints2, descriptors2);

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

void RegistroRGB::listFile(string direc, vector<string> *files_lt)
{
    DIR *pDIR;
    struct dirent *entry;

    if( pDIR=opendir(direc.c_str()) ) {
        while(entry = readdir(pDIR)) {
            if(strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 ) {
                string str = entry->d_name;
                files_lt->push_back(direc+str);
            }
        }
    }
    closedir(pDIR);
    sort(files_lt->begin(), files_lt->end());
}

