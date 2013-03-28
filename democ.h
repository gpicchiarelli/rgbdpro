/**
 * File: democ.h
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
#ifndef DEMOC_H
#define DEMOC_H

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/features2d.hpp>

#include <boost/thread/thread.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/ordered_index.hpp>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

// DBoW2
#include "DBoW2.h"
#include "DUtils/DUtils.h"
#include "DUtilsCV/DUtilsCV.h"
#include "DVision/DVision.h"

//yaml-cpp
#include "yaml-cpp/yaml.h"


#include "registrorgb.h"
#include "registro3d.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>

using namespace std;
using namespace DBoW2;
using namespace DUtils;
using namespace cv;


vector<string> files_list_rgb,files_list_3d;
map<string, string> registro_interno;

typedef pair <string, string> PairString;
typedef vector<vector<vector<float> > > BoWFeatures;

//avoid boost:: invalid free
struct null_deleter
{
    void operator()(void const *) const
    {
    }
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void loopClosingRGB(BoWFeatures &features);
void loopClosing3d(BoWFeatures &features);
void loadFeatures3d(BoWFeatures &features);
void loadFeaturesRGB(BoWFeatures &features);
void testVocCreation(BoWFeatures &features,BoWFeatures &featuresrgb);

void searchRegistro();
void listFile(string direc, vector<string> *files_lt);
void changeStructure(const vector<float> &plain, vector<vector<float> > &out,int L);
void readPoseFile(const char *filename,  vector<double> &xs,  vector<double> &ys);
void wait();
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void wait()
{
    cout << "Premi 'enter' per continuare." << endl;
    getchar();
}

void listFile(string direc, vector<string> *files_lt)
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
void readPoseFile(const char *filename,  vector<double> &xs,  vector<double> &ys)
{
    xs.clear();
    ys.clear();
    
    string line;
    ifstream Myfile;
    Myfile.open(filename);
    if(Myfile.is_open()) {
        while(Myfile.peek() != EOF) {
            vector<string> w1;
            getline(Myfile, line);
            StringFunctions::trim(line);
            if(line.size() > 0) {
                //luogo
                StringFunctions::split(line,w1,",");
                string w = w1[1];
                StringFunctions::trim(w);
                string h = w1[2];
                xs.push_back(boost::lexical_cast<double>(w));
                ys.push_back(boost::lexical_cast<double>(h));
            }
        }
        Myfile.close();
    }
}
void searchRegistro()
{
    string line;
    ifstream Myfile;
    Myfile.open("registro.txt");
    if(Myfile.is_open()) {
        while(Myfile.peek() != EOF) {
            vector<string> w1,w2;
            getline(Myfile, line);
            StringFunctions::trim(line);
            if(line.size() > 0) {
                //luogo
                StringFunctions::split(line,w1,":");
                string w = w1[1];
                StringFunctions::trim(w);
                string h = w1[0];
                //nome immagine
                StringFunctions::split(h,w2,"."); //prendo solo il nome file
                h = w2[0];
                StringFunctions::trim(h);
                registro_interno.insert(PairString(h,w));
            }
        }
        Myfile.close();
    } else {
        cout<<"registro.txt NON TROVATO."<<endl;
    }
}
void changeStructure(const vector<float> &plain, vector<vector<float> > &out,int L)
{
    out.resize(plain.size() / L);
    int j = 0;
    for(int i = 0; i < plain.size(); i += L, ++j)
    {
        out[j].resize(L);
        copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
    }
}

#endif // DEMOC_H
