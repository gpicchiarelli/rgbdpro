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

#include "democ.h"

//impostazioni utili al debug
#define DEBUG 0 

string filename_voc_3d = "voc_3d.yml.gz";
string filename_voc_rgb = "voc_rgb.yml.gz";

RegistroRGB* reg_RGB;
Registro3D* reg_3D;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
string directory3d = "pcd/";
string directory3dbin = "pcdbin/";
string directoryrgb = "_images/";
string debug_directory3d = "debug_pcd/";
string debug_directoryrgb = "debug_images/";

int main(int nNumberofArgs, char* argv[])
{    
    BoWFeatures featuresrgb,features3d;
    bool flag_voc=false, flag_debug=false,flag_loop = false,flag_bin = false,flag_s=false,flag_u=false;
    bool flag_l3d=false,flag_lrgb=false;
    if(nNumberofArgs > 0){
        vector<string> parametri;
        for(int y = 0 ; y < nNumberofArgs; y++)
        {
            string parm = argv[y];
            StringFunctions::trim(parm);
            parametri.push_back(parm);
        }
        if ( find(parametri.begin(), parametri.end(), "-U") != parametri.end()){
            flag_u= true;
            cout << "--- OPZIONE UTILIZZA FEATURES SALVATE ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-S") != parametri.end()){
            flag_s= true;
            cout << "--- OPZIONE SALVA FEATURES IN FORMATO BINARIO ---" << endl;
        }
        
        if ( find(parametri.begin(), parametri.end(), "-b") != parametri.end()){
            flag_bin= true;
            cout << "--- OPZIONE DEPTH IN FORMATO BINARIO ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-l") != parametri.end()){
            flag_loop= true;
            cout << "--- OPZIONE LOOP CLOSING RGB-DEPTH ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-l-rgb") != parametri.end()){
            flag_lrgb = true;
            flag_loop = false;
            cout << "--- OPZIONE LOOP CLOSING RGB---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-l-3d") != parametri.end()){
            flag_l3d = true;
            flag_loop = false;
            cout << "--- OPZIONE LOOP CLOSING DEPTH ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-vdel") != parametri.end()){
            flag_voc= true;
            cout << "--- OPZIONE VOCABOLARIO ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-D") != parametri.end()){
            flag_debug= true;
            cout << "--- OPZIONE DEBUG ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-h") != parametri.end()){
            cout << endl << "DBOW NARF+SURF128 test: " << endl;
            cout << endl;
            cout <<"Nome Vocabolario NARF: "<< filename_voc_3d<< endl;
            cout <<"Nome Vocabolario SURF128: "<< filename_voc_rgb<< endl;
            cout << endl;
            cout << "[ -h ]: Guida" << endl;
            cout << "[ -vdel ]: Cancella i vecchi vocabolari." << endl;
            cout << "[ -l ]: Loop Closing." << endl;
            cout << "[ -l-rgb ]: Loop Closing RGB." << endl;
            cout << "[ -l-3d ]: Loop Closing RGB." << endl;
            cout << "[ -b ]: Si utilizzano il file depth in formato binario." << endl;
            cout << "[ -S ]: Salva DATABASE ed esce." << endl;
            cout << "[ -D ]: Modalità DEBUG. Dataset ridotto cartelle debug_{*}" << endl;
            exit(0);
        }else{
            if (flag_voc || flag_loop || flag_s || flag_u){
                searchRegistro();      
                
                if (flag_debug){
                    directory3d = debug_directory3d;
                    directoryrgb = debug_directoryrgb;
                }
                if(flag_bin && !flag_debug){
                    directory3d = directory3dbin;
                }
                
                reg_3D = new Registro3D(directory3d);
                reg_RGB = new RegistroRGB(directoryrgb);
                
                listFile(directoryrgb,&files_list_rgb);
                listFile(directory3d,&files_list_3d);
                
                if(flag_voc){
                    if( remove(filename_voc_3d.c_str()) != 0 )
                        perror( "Errore cancellazione vocabolario NARF" );
                    else
                        puts( "Vocabolario NARF cancellato." );
                    
                    if( remove( filename_voc_rgb.c_str() ) != 0 )
                        perror( "Errore cancellazione vocabolario SURF" );
                    else
                        puts( "Vocabolario SURF cancellato." );
                }      
                if(flag_u){
                    loadFeaturesFile(features3d,"feat_3d.dat");
                    loadFeaturesFile(featuresrgb,"feat_rgb.dat");
                }else{
                    if(!flag_s){
                        loadFeaturesRGB(featuresrgb);                    
                        loadFeatures3d(features3d);
                    }
                }
                if(flag_s){
                    
                    loadFeaturesRGB(featuresrgb);  
                    saveFeaturesFile(featuresrgb,"feat_rgb.dat");
                    featuresrgb.clear();
                    
                    loadFeatures3d(features3d);
                    saveFeaturesFile(features3d,"feat_3d.dat"); 
                    features3d.clear();
                    
                    testVocCreation(features3d,featuresrgb);
                    return 0;
                }
                
                testVocCreation(features3d,featuresrgb);
                if(flag_loop){
                    loopClosingRGB(featuresrgb);
                    loopClosing3d(features3d);
                }else{
                    if(flag_l3d){loopClosing3d(features3d);}
                    if(flag_lrgb){loopClosingRGB(featuresrgb);}
                }
            }else{
                cout << "Errore: nessun parametro. Per la guida usare parametro '-h' " << endl;
            }
        }
    }
    else
    {
        cout << "Errore: nessun parametro. Per la guida usare parametro '-h' " << endl;
    }
    cout << "DBoW2 TEST: terminato." << endl;
    return 0;
}

void loopClosingRGB(BoWFeatures &features){
    
    int const INITIAL_OFFSET = 80; 
    double const MATCH_THRESHOLD = 0.47; 
    double const SANITY_THRESHOLD = 0.6;
    int const END_OFFSET = INITIAL_OFFSET;
    int const SIZE_BUCKET = 5; 
    int const TEMP_CONS = 6;
    bool loop_found = false;
    
    // load robot poses
    vector<double> xs, ys;
    readPoseFile("g_truth_rgb.txt", xs, ys);
    cout << "RGB: Acquisizione Ground Truth" << endl;
    
    // prepare visualization windows
    DUtilsCV::GUI::tWinHandler win = "Immagine Analizzata";
    DUtilsCV::GUI::tWinHandler winplot = "TraiettoriaRGB";
    
    DUtilsCV::Drawing::Plot::Style normal_style(2); // thickness
    DUtilsCV::Drawing::Plot::Style loop_style('r', 2); // color, thickness
    
    DUtilsCV::Drawing::Plot implot(240, 320,
                                   - *std::max_element(xs.begin(), xs.end()),
                                   - *std::min_element(xs.begin(), xs.end()),
                                   *std::min_element(ys.begin(), ys.end()),
                                   *std::max_element(ys.begin(), ys.end()), 25);
    
    Surf128Vocabulary voc(filename_voc_rgb);
    Surf128Database db(voc, false);
    
    vector<int> bucket;
    if (!DEBUG) { wait(); }
    
    for (int i=0;i<files_list_rgb.size();i++){
        loop_found = false;
        Mat im = reg_RGB->getImageAt(i);
        DUtilsCV::GUI::showImage(im, true, &win, 10);
        if ((i+1)>INITIAL_OFFSET){            
            BowVector v1, v2;
            voc.transform(features[i], v1);                
            voc.transform(features[i-1], v2);   
            cout << "SANITY:" << voc.score(v1,v2) <<endl;
            if (voc.score(v1,v2) >= SANITY_THRESHOLD){           
                QueryResults ret;
                db.query(features[i], ret,db.size());
                for (int j = 0; j < ret.size(); j++){            
                    if (ret[j].Score > MATCH_THRESHOLD){ 
                        if ((i - END_OFFSET) >= ret[j].Id){
                            if (bucket.size() < SIZE_BUCKET){
                                bucket.push_back(ret[j].Id);
                                cout << i <<"-> bucket: " << ret[j].Id<<endl;
                            }
                            else
                            {                            
                                break;   
                            }
                        }
                    }
                }            
                ret.clear();
                if (bucket.size()>0){
                    cout << "---- TEMPORAL CONSISTENCY {"<<i<< "} ----" << endl;                    
                    BowVector v1, v2;                
                    for (int aa = 0 ; aa < bucket.size(); aa++){                    
                        for (int xx = 0;xx< TEMP_CONS;xx++){
                            int pivot = xx+1;
                            int cursore = bucket[aa] - pivot; 
                            if(cursore >= 0){
                                voc.transform(features[i-pivot], v1);                        
                                voc.transform(features[cursore], v2);
                                double score = voc.score(v1, v2);   
                                cout << (i-pivot) << " vs " << cursore << " score: " << score<<endl;
                                if (!(score >= MATCH_THRESHOLD)){               
                                    bucket[aa] = -1;
                                    break;
                                }
                            }else{
                                bucket[aa] = -1;
                                break;
                            }
                        }
                    }
                }                
                int maxInliers = 0;
                int maxIdInliers = -1;
                int tyu = 0;                
                for(int yy = 0; yy < bucket.size(); yy++){
                    if(bucket[yy] != -1) {
                        tyu = reg_RGB->inliersRGB(i,bucket[yy]);
                        if (maxInliers < tyu){
                            maxInliers = tyu;
                            maxIdInliers = bucket[yy];
                        }
                    }
                }
                if(maxIdInliers != -1){     
                    loop_found = true;
                    cout << "LOOP TROVATO - INLIERS MAX:" << maxInliers << ". "<< i <<" per immagine: " << maxIdInliers << endl;
                    implot.line(-xs[maxIdInliers], ys[maxIdInliers], -xs[i], ys[i], loop_style);
                }
                bucket.clear();           
            }
        }
        db.add(features[i]);       
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }    
}

void loopClosing3d(BoWFeatures &features)
{    
    int const INITIAL_OFFSET = 80; 
    double const SANITY_THRESHOLD = 0.74;
    double const MATCH_THRESHOLD = 0.73;
    int const END_OFFSET = INITIAL_OFFSET;
    int const SIZE_BUCKET = 5;
    int const TEMP_CONS = 6;
    bool loop_found = false;
    
    if (!DEBUG) { wait(); }
    vector<int> bucket;
    vector<double> xs, ys;
    
    readPoseFile("g_truth_3d.txt", xs, ys);
    cout << "3D: Acquisizione Ground Truth" << endl;
    
    DUtilsCV::GUI::tWinHandler winplot = "Traiettoria3D";
    
    DUtilsCV::Drawing::Plot::Style normal_style(2); // thickness
    DUtilsCV::Drawing::Plot::Style loop_style('r', 2); // color, thickness
    
    DUtilsCV::Drawing::Plot implot(240, 320,
                                   - *std::max_element(xs.begin(), xs.end()),
                                   - *std::min_element(xs.begin(), xs.end()),
                                   *std::min_element(ys.begin(), ys.end()),
                                   *std::max_element(ys.begin(), ys.end()), 25);
    
    
    NarfVocabulary voc(filename_voc_3d);
    NarfDatabase db(voc, false);    
    
    for (int i=0;i<files_list_3d.size();i++){
        loop_found = false;
        if ((i+1)>INITIAL_OFFSET){            
            BowVector v1, v2;
            voc.transform(features[i], v1);                
            voc.transform(features[i-1], v2);
            cout << "SANITY:" << voc.score(v1,v2) <<endl;
            if (voc.score(v1,v2) >= SANITY_THRESHOLD){           
                QueryResults ret;
                db.query(features[i], ret,db.size());
                for (int j = 0; j < ret.size(); j++){ //scansiono risultati                
                    if (ret[j].Score > MATCH_THRESHOLD){ //sanity check
                        if ((i - END_OFFSET) >= ret[j].Id){ //scarto la coda
                            if (bucket.size() < SIZE_BUCKET){
                                bucket.push_back(ret[j].Id);
                                cout << i <<"-> bucket: " << ret[j].Id<<endl;
                            }
                            else
                            {                            
                                break;   
                            }
                        }
                    }
                }            
                ret.clear();
                if (bucket.size()>0){
                    cout << "---- TEMPORAL CONSISTENCY {"<<i<< "} ----" << endl;                    
                    BowVector v1, v2;                
                    for (int aa = 0 ; aa < bucket.size(); aa++){                    
                        for (int xx = 0;xx< TEMP_CONS;xx++){
                            int pivot = xx+1;
                            int cursore = bucket[aa] - pivot;    
                            if (cursore>=0){
                                voc.transform(features[i-pivot], v1);                        
                                voc.transform(features[cursore], v2);
                                double score = voc.score(v1, v2);   
                                cout << (i-pivot) << " vs " << cursore << " score: " << score<<endl;
                                if (!(score >= MATCH_THRESHOLD)){               
                                    bucket[aa] = -1;
                                    break;
                                }
                            }else{
                                bucket[aa] = -1;
                                break;
                            }
                        }
                    }
                }
                
                double maxInliers = numeric_limits<double>::max();
                int maxIdInliers = -1;
                int tyu = 0;                
                for(int yy = 0; yy < bucket.size(); yy++){
                    if(bucket[yy] != -1) {
                        tyu = reg_3D->getScoreFit(i,bucket[yy]);
                        if (maxInliers > tyu){
                            maxInliers = tyu;
                            maxIdInliers = bucket[yy];
                        }
                    }
                }
                if(maxIdInliers != -1){     
                    loop_found = true;
                    cout << "LOOP TROVATO - INLIERS MAX:" << maxInliers << ". "<< i <<" per immagine: " << maxIdInliers << endl;
                    implot.line(-xs[maxIdInliers], ys[maxIdInliers], -xs[i], ys[i], loop_style);
                }
                bucket.clear();           
            }
        }
        db.add(features[i]);       
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }    
}

void loadFeaturesRGB(BoWFeatures &features)
{
    features.clear();
    features.reserve(files_list_rgb.size());
    cv::SURF surf(300, 5, 4, true);
    
    
    for (int i = 0; i < files_list_rgb.size(); ++i) {
        cout << "Estrazione SURF: " << files_list_rgb[i];
        
        cv::Mat image = cv::imread(files_list_rgb[i]);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        vector<float> descriptors;
        surf(image, mask, keypoints, descriptors);
        features.push_back(vector<vector<float> >());
        changeStructure(descriptors, features.back(), surf.descriptorSize());
        cout << ". Estratti " << features[i].size() << " descrittori." << endl;
        descriptors.clear();
        keypoints.clear();
        mask.release();
        image.release();
    }
    cout << "Estrazione terminata." << endl;
}

// ----------------------------------------------------------------------------
void loadFeatures3d(BoWFeatures &features)
{
    typedef pcl::PointXYZ PointType;
    float angular_resolution = pcl::deg2rad (0.2);
    float support_size = 0.1f;
    
    features.clear();
    features.reserve(files_list_3d.size());
    
    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 1;
    
    for (int i = 0; i < files_list_3d.size(); ++i) {     
        pcl::PointCloud<PointType>::Ptr point_cloud_wf (new pcl::PointCloud<PointType>);        
        pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
        
        pcl::io::loadPCDFile (files_list_3d[i], *point_cloud_wf);
        
        //filtraggio valori NaN
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud (*point_cloud_wf,*point_cloud_wf,indices);
        //filtraggio con pixel volumetrici
        const float VOXEL_GRID_SIZE = 0.01f;
        pcl::VoxelGrid<PointType> vox_grid;
        vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
        vox_grid.setInputCloud(point_cloud_wf);
        vox_grid.filter(*point_cloud);
        
        cout << "Estrazione NARF: " << files_list_3d[i] ;
        
        Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f ((*point_cloud).sensor_origin_[0],
                                             (*point_cloud).sensor_origin_[1],
                (*point_cloud).sensor_origin_[2])) *
                Eigen::Affine3f ((*point_cloud).sensor_orientation_);
        
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;      
        pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage,null_deleter());
        pcl::RangeImage& range_image = *range_image_ptr;
        range_image.createFromPointCloud ((*point_cloud),angular_resolution,pcl::deg2rad(360.0f),pcl::deg2rad(180.0f),scene_sensor_pose,coordinate_frame,noise_level,min_range,border_size);
        range_image.setUnseenToMaxRange();
        
        pcl::RangeImageBorderExtractor range_image_border_extractor;
        pcl::NarfKeypoint narf_keypoint_detector;
        narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
        narf_keypoint_detector.setRangeImage (&range_image);
        narf_keypoint_detector.getParameters().support_size = support_size;
        
        //euristiche, per avvicinarsi al real time
        narf_keypoint_detector.getParameters().max_no_of_threads = 3;
        narf_keypoint_detector.getParameters().calculate_sparse_interest_image=false;
        narf_keypoint_detector.getParameters().use_recursive_scale_reduction=true;
        
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
        cout << ". Estratti "<<narf_descriptors.size ()<<" descrittori. Punti: " <<keypoint_indices.points.size ()<< "."<<endl;
        features.push_back(vector<vector<float> >());
        
        for (int p = 0; p < narf_descriptors.size(); p++) {
            vector<float> flot;
            copy(narf_descriptors[p].descriptor, narf_descriptors[p].descriptor+FNarf::L, back_inserter(flot));
            features.back().push_back(flot);
            flot.clear();
        }
        indices.clear();    
        range_image_border_extractor.clearData();        
        narf_keypoint_detector.clearData();
        (*range_image_ptr).clear();
        keypoint_indices.clear();
        keypoint_indices2.clear();
        (*point_cloud).clear();
        (*point_cloud_wf).clear();
        range_image.clear();
        narf_descriptors.clear();
        narf_descriptor = NULL;
    }
    cout << "Estrazione terminata." << endl;
}

void testVocCreation(BoWFeatures &features,BoWFeatures &featuresrgb)
{ 
    // branching factor and depth levels
    const int k = 10;
    const int L = 3;
    const ScoringType score = L1_NORM;
    const WeightingType weight = TF_IDF;
    
    NarfVocabulary voc;
    Surf128Vocabulary voc2;
    
    string nomefile_3d = filename_voc_3d;
    string nomefile_rgb = filename_voc_rgb;
    
    cv::FileStorage fs(nomefile_3d.c_str(), cv::FileStorage::READ);
    cv::FileStorage fs2(nomefile_rgb.c_str(), cv::FileStorage::READ);
    
    if ((!fs.isOpened()) || (!fs2.isOpened()))
    {
        NarfVocabulary voc(k, L, weight, score);
        voc.create(features);
        Surf128Vocabulary voc2(k, L, weight, score);
        voc2.create(featuresrgb);
        
        cout << "Creazione " << k << "^" << L << " vocabolario 3D terminata." << endl;
        cout << "Vocabolario 3D: " << endl << voc << endl << endl;
        
        cout << "Creazione " << k << "^" << L << " vocabolario RGB terminata." << endl;
        cout << "Vocabolario RGB: " << endl << voc2 << endl << endl;
        
        voc.save(nomefile_3d);
        voc2.save(nomefile_rgb);
    }
    else 
    {
        voc.load(nomefile_3d);
        voc2.load(nomefile_rgb);
        cout << "Vocabolario 3D: " << endl << voc << endl << endl;
        cout << "Vocabolario RGB: " << endl << voc2 << endl << endl;
    }
    
    //    BowVector v1, v2;
    //    ofstream bown("bow.txt");
    //    const static int taglia = (pow(k,L)*2) + 1; //il +1 si riferisce all'etichetta del luogo.
    //    const static int offset = taglia/2;
    
    //    for(int i ; i < files_list_3d.size(); i++)
    //    {
    //        voc.transform(features[i], v1);
    //        voc2.transform(featuresrgb[i], v2);
    
    //        double bbow[taglia];
    //        for(int jay = 0; jay < taglia;jay++){
    //            bbow[jay] = 0;
    //        }
    //        //RGB
    //        for(BowVector::iterator vit = v2.begin(); vit != v2.end(); vit++){
    //            int pivot = vit->first;
    //            bbow[pivot] = vit->second;
    //        }
    //        //3D
    //        for(BowVector::iterator vit = v1.begin(); vit != v1.end(); vit++){
    //            int pivot = offset + vit->first;
    //            bbow[pivot] = vit->second;
    //        }
    
    //        //inserimento etichetta luogo per generare il file che verrà inviato all'apprendimento.
    //        string luogo = registro_aux.at(i);
    //        vector<string> a;
    //        boost::split(a, registro_aux.at(i), boost::is_any_of("=>"));
    //        a.erase(  remove_if( a.begin(), a.end(), boost::bind( & string::empty, _1 ) ), a.end());
    //        StringFunctions::trim(a[1]);
    //        bown << a[1] << ",";
    
    //        for(int jay = 0; jay < taglia;jay++){
    //            bown << bbow[jay];
    //            if (jay < taglia - 1){ bown << ","; }
    //        }
    //        bown << endl;
    //    }
    //    bown.close();
}
