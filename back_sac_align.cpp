const double NORMALS_RADIUS = 0.02;
const double FEATURES_RADIUS = 0.04; //0.08
const double SAC_MAX_CORRESPONDENCE_DIST = 1;
const double SAC_MIN_SAMPLE_DIST = 0.05; 

pcl::PointCloud<pcl::Normal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::Normal>);

pcl::NormalEstimation<PointXYZ, pcl::Normal> norm_est;
pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
norm_est.setSearchMethod (tree);
norm_est.setRadiusSearch( NORMALS_RADIUS );
norm_est.setInputCloud (point_cloud_wf);
norm_est.compute (*points_with_normals_src);
pcl::copyPointCloud (*point_cloud_wf, *points_with_normals_src);

norm_est.setInputCloud (point_cloud_prec);
norm_est.compute (*points_with_normals_tgt);
pcl::copyPointCloud(*point_cloud_prec, *points_with_normals_tgt);

cout<< "Normals estimated." <<endl;

pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_wf;
pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_prec;

fpfh_wf.setInputCloud (point_cloud_wf);
fpfh_prec.setInputCloud (point_cloud_prec);            
fpfh_wf.setInputNormals (points_with_normals_src);
fpfh_prec.setInputNormals (points_with_normals_tgt); 
fpfh_wf.setRadiusSearch( FEATURES_RADIUS ); 
fpfh_prec.setRadiusSearch( FEATURES_RADIUS ); 
fpfh_wf.setSearchMethod (tree);
fpfh_prec.setSearchMethod(tree);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr frame_features (new pcl::PointCloud<pcl::FPFHSignature33> ());

fpfh_wf.compute (*frame_features);
fpfh_prec.compute (*model_features); 

cout<< "FPFH estimated." <<endl;

//Initialize alignment method
pcl::SampleConsensusInitialAlignment<PointXYZ, PointXYZ, pcl::FPFHSignature33> sac_ia;
sac_ia.setInputCloud( point_cloud_wf );
sac_ia.setSourceFeatures( frame_features );
sac_ia.setInputTarget( point_cloud_prec);
sac_ia.setTargetFeatures( model_features );

//Set parameters for alignment and RANSAC
sac_ia.setMaxCorrespondenceDistance(SAC_MAX_CORRESPONDENCE_DIST);
sac_ia.setMinSampleDistance( SAC_MIN_SAMPLE_DIST );
sac_ia.setMaximumIterations(10);            
//align frame using FPFH features
sac_ia.align( *point_cloud_wf );  

cout << "SAC align. OK." << endl;

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setMaxCorrespondenceDistance (0.01);
icp.setRANSACOutlierRejectionThreshold (0.01);
icp.setTransformationEpsilon (0.00001); //e-8
icp.setMaximumIterations (10);
icp.setEuclideanFitnessEpsilon (0.1); 
icp.setInputCloud (point_cloud_wf);
icp.setInputTarget (point_cloud_prec);            
icp.align ((*point_cloud_a));  
//pcl::transformPointCloud(*point_cloud_a,*point_cloud_a,icp.getFinalTransformation());
//cout << icp.getFinalTransformation() << endl;

 cout << "ICP align. OK." << endl;
