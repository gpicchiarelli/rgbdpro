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


#include "registro3d.h"

Registro3D::Registro3D(int size){
    this->_reg_int_names.reserve(size);
    this->_reg_int_points.reserve(size);
    this->_reg_int_descri.reserve(size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned___ (new pcl::PointCloud<pcl::PointXYZ>());
    this->aligned = aligned___;
}

void Registro3D::addFrame(string name,pcl::PointCloud<pcl::PointXYZ> points, pcl::PointCloud<pcl::Narf36> narf_descriptors)
{
    this->_reg_int_names.push_back(name);
    this->_reg_int_points.push_back(points);
    this->_reg_int_descri.push_back(narf_descriptors);
}

double Registro3D::getScoreFit(int src_p, int dst_p)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    *src_ptr = this->_reg_int_points[src_p];

    pcl::PointCloud<pcl::PointXYZ>::Ptr dst_ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    *dst_ptr = this->_reg_int_points[dst_p];

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance (0.05);
    icp.setRANSACOutlierRejectionThreshold (0.1);
    icp.setTransformationEpsilon (1e-8);
    icp.setMaximumIterations (20);
    icp.setInputCloud (src_ptr);
    icp.setInputTarget (dst_ptr);
    icp.align (*(this->aligned));
    this->refined_T = icp.getFinalTransformation ();
    double res = icp.getFitnessScore();

    return res;
}
