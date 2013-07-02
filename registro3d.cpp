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
    this->__directory = directory;
    this->listFile(directory,&this->__files_list_3d);
}

/* method untested http://www.pcl-users.org/Feature-correspondence-using-NARF-descriptors-td2444164.html */
float Registro3D::getCorrispondences(int src_p,int dst_p){
    PointCloud<Narf36> n1,n2;
    n1 = this->extractNARF(src_p);
    n2 = this->extractNARF(dst_p);
    
    KdTreeFLANN<Narf36> kdtree;
	kdtree.setPointRepresentation (boost::make_shared <NARFPointRepresenation>());
  	PointCloud<Narf36>::Ptr narf_descriptors2Ptr (&n2);
	kdtree.setInputCloud(narf_descriptors2Ptr) ;
	int k = 20 ; //Find k nearset neighbours 
	vector<int> k_indices (k); //k matched indices
  	vector<float> k_distances (k); //distances of k matched indices
	kdtree.nearestKSearch (n1.points[10], k, k_indices, k_distances);
	cout<<"k_indices k_distances"<<endl;
	for (int i=0; i < k; ++i)
	{
		cout<<k_indices[i]<<" "<<k_distances[i]<<endl ;
	}
    return k_indices.size();
}


PointCloud<pcl::Narf36> Registro3D::extractNARF(int i){  

    typedef pcl::PointXYZ PointType;
    float angular_resolution = pcl::deg2rad (0.3f);
    float support_size = 0.1f;
    
    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 1;
    
    //pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    
    pcl::PointCloud<PointType>::Ptr point_cloud_wf (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
    
    pcl::io::loadPCDFile (this->__files_list_3d[i], *point_cloud_wf);
    
    //filtraggio valori NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*point_cloud_wf,*point_cloud_wf,indices);
    
    const float VOXEL_GRID_SIZE = 0.01f;
    pcl::VoxelGrid<PointType> vox_grid;
    vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
    vox_grid.setInputCloud(point_cloud_wf);
    vox_grid.filter(*point_cloud);
    
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f ((*point_cloud).sensor_origin_[0],
                                         (*point_cloud).sensor_origin_[1],
            (*point_cloud).sensor_origin_[2])) *
            Eigen::Affine3f ((*point_cloud).sensor_orientation_);
    
    range_image.max_no_of_threads = 2;
    range_image.createFromPointCloud ((*point_cloud),angular_resolution,pcl::deg2rad(360.0f),pcl::deg2rad(180.0f),scene_sensor_pose,coordinate_frame,noise_level,min_range,border_size);
    range_image.setUnseenToMaxRange();
    
    //range_image_widget.showRangeImage (range_image);
    
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;
    //euristiche, per avvicinarsi al real time
    narf_keypoint_detector.getParameters().max_no_of_threads = 2;
    narf_keypoint_detector.getParameters().calculate_sparse_interest_image=false; //false
    narf_keypoint_detector.getParameters().use_recursive_scale_reduction=false; //true
    //narf_keypoint_detector.getParameters().add_points_on_straight_edges=true;
    
    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    
    vector<int> keypoint_indices2;
    keypoint_indices2.resize (keypoint_indices.points.size ());
    for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
        keypoint_indices2[i]=keypoint_indices.points[i];
    pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
    narf_descriptor.getParameters().support_size = support_size;
    narf_descriptor.getParameters().rotation_invariant = true;
    pcl::PointCloud<pcl::Narf36> narf_descriptors;
    
    narf_descriptor.compute (narf_descriptors);
    return narf_descriptors;    
}

void Registro3D::addFrame(PointCloud<Narf36> frame,string name)
{
    string gg = (this->__directory) + "3d_descriptors/"+name;
    pcl::io::savePCDFileBinary(gg.c_str(),frame);
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

double Registro3D::getCentroidDiff(int src_p,int dst_p){
    Eigen::Vector4f cstart, cend;
    const float VOXEL_GRID_SIZE = 0.02;
    //TODO use pose of scan

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
    
    icp.setMaxCorrespondenceDistance (0.1);
    icp.setRANSACOutlierRejectionThreshold (0.5);
    icp.setTransformationEpsilon (1e-3); //e-8
    icp.setMaximumIterations (4);
    icp.setInputCloud (src_ptr_f);
    icp.setInputTarget (dst_ptr_f);
    icp.align (*(this->aligned));
    
    pcl::compute3DCentroid (*src_ptr_f, cstart);
    pcl::compute3DCentroid (*(this->aligned), cend);
    Eigen::Vector4f diff = cend - cstart;
cout << diff.norm() << endl;
    return diff.norm();
}

double Registro3D::getScoreFit(int src_p, int dst_p)
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
    
    //units are meters 0.01
    const float VOXEL_GRID_SIZE = 0.08;
    
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
    
    icp.setMaxCorrespondenceDistance (0.1);
    icp.setRANSACOutlierRejectionThreshold (0.5);
    icp.setTransformationEpsilon (1e-3); //e-8
    icp.setMaximumIterations (4);
    icp.setInputCloud (src_ptr_f);
    icp.setInputTarget (dst_ptr_f);
    icp.align (*(this->aligned));

    this->refined_T = icp.getFinalTransformation ();
    (*this->aligned).clear();
    
    return icp.getFitnessScore();
}

double Registro3D::getScoreFix(vector<vector<float> > src, vector<vector<float> > dst)
{
    /*
    for()
    boost::lexical_cast<double>(pcl::L1_Norm(src[],dst[],36));
    */
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
