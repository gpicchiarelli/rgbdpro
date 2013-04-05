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
#include "vtkProperty.h"


Registro3D::Registro3D(string directory){
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned___ (new pcl::PointCloud<pcl::PointXYZ>());
    this->aligned = aligned___;
    this->listFile(directory,&this->__files_list_3d);
}

pcl::RangeImage Registro3D::getRangeImageAt(int position){

    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile (this->__files_list_3d[position], *point_cloud);

    //filtraggio valori NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*point_cloud,*point_cloud,indices);

    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
    //SCENE_SENSOR_POSE
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f ((*point_cloud).sensor_origin_[0],
                                                               (*point_cloud).sensor_origin_[1],
                                                               (*point_cloud).sensor_origin_[2])) *
            Eigen::Affine3f ((*point_cloud).sensor_orientation_);
    range_image.createFromPointCloud ((*point_cloud), pcl::deg2rad (0.1), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.setUnseenToMaxRange();
    return range_image;
}

double Registro3D::getScoreFit(int src_p, int dst_p)
{
    //units are meters
    const float VOXEL_GRID_SIZE = 0.001;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile (this->__files_list_3d[src_p], *src_ptr);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*src_ptr,*src_ptr,indices);
    indices.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr dst_ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile (this->__files_list_3d[dst_p], *dst_ptr);

    pcl::removeNaNFromPointCloud (*dst_ptr,*dst_ptr,indices);

    //voxelgrid
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr_f( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr dst_ptr_f( new pcl::PointCloud<pcl::PointXYZ>() );

    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
    vox_grid.setInputCloud( src_ptr );
    vox_grid.filter( *src_ptr_f );

    vox_grid.setInputCloud( dst_ptr );
    vox_grid.filter( *dst_ptr_f );

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance (1);
    icp.setRANSACOutlierRejectionThreshold (0.3);
    icp.setTransformationEpsilon (4); //e-8
    icp.setMaximumIterations (5);
    icp.setInputCloud (src_ptr_f);
    icp.setInputTarget (dst_ptr_f);

//    this->registrationVisualizer.startDisplay();
//    this->registrationVisualizer.setMaximumDisplayedCorrespondences (100);
//    this->registrationVisualizer.setRegistration (icp);

    icp.align (*(this->aligned));
    this->refined_T = icp.getFinalTransformation ();
    double res ;
    res = 0;
    if (icp.hasConverged()){
        res = icp.getFitnessScore();
    }else{
        res= 0;
        cout<< "ICP: NO CONVERGENZA" << endl;
    }
    cout << "FitnessScore: " << res << endl;
    return res;
}

void Registro3D::listFile(string direc, vector<string> *files_lt)
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
