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
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>



class Registro3D {
    typedef pcl::PointCloud<pcl::PointXYZ> p_cloud;
    typedef pcl::PointCloud<pcl::Narf36> p_narf;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Registro3D(int size);
        double getScoreFit(int src,int dst);
        void addFrame(string name,pcl::PointCloud<pcl::PointXYZ> points, pcl::PointCloud<pcl::Narf36> narf_descriptors);
    private:
        vector<string> _reg_int_names;
        vector<p_cloud> _reg_int_points;
        vector<p_narf> _reg_int_descri;
        Eigen::Matrix4f refined_T;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned;
};



#endif // REGISTRO3D_H
