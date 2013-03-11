/* Giacomo Picchiarelli */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <string.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>

#include <boost/algorithm/string.hpp>
using namespace std;
using namespace boost;
using namespace pcl;

void listFile(string direc, vector<string> *files_lt);
vector<string> files_list_3d;

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string direc_text = "pcd/";
    string direc_bin = "pcdbin/";

    listFile(direc_text,&files_list_3d);

    for(int yy = 0; yy < files_list_3d.size(); yy ++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (files_list_3d[yy], *cloud) == -1) //* load the file
        {
            cout << "Errore: " << files_list_3d[yy] << endl;
            return -1;
        }
        else
        {
            cout << "Leggo: " << files_list_3d[yy] << endl;
            string wrk = boost::ireplace_first_copy(files_list_3d[yy],"pcd/","pcdbin/");
            pcl::io::savePCDFileBinary (wrk, *cloud);
        }
    }
    return 0;
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
