/**
 * File: registro3d
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

#ifndef REGISTRO3D_H
#define REGISTRO3D_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>
#include <string.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
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
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/registration_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/impl/point_types.hpp>

#include <boost/thread/thread.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp> 
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/export.hpp> 
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/serialization/static_warning.hpp>

#include "DBoW2.h"

using namespace pcl;

typedef PointXYZ PointType;

// Define new PointRepresentation for NARF36
class NARFPointRepresenation : public PointRepresentation<Narf36>
{
public:
  NARFPointRepresenation () 
  { 
    this->nr_dimensions_ = 36; 
  }
  
  void copyToFloatArray (const Narf36 &p, float *out) const
  {
    for(int i=0; i < 36; ++i)
    	out[i] = p.descriptor[i];
  }
};

class Registro3D {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Registro3D(string directory);
        double getScoreFit(int src,int dst);
        double getScoreFix(vector<vector<float> > src, vector<vector<float> > dst);        
        pcl::PointCloud<pcl::PointXYZ> getTra(int src_p, int dst_p);
        pcl::RangeImage getRangeImageAt(int position);
        float getCorrispondences(int src,int dst);
        void addFrame(PointCloud<Narf36> frame,string name);
        PointCloud<pcl::Narf36> extractNARF(int i);
    private:
        Eigen::Matrix4f refined_T;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned;
        vector<string> __files_list_3d;
        void listFile(string direc, vector<string> *files_lt);
        pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;
        string __directory;
};



#endif // REGISTRO3D_H
