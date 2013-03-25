/**
 * File: democ.cpp
 * Date: November 2012
 * Author: Giacomo Picchiarelli <gpicchiarelli@gmail.com>
 * Description: test NARF - SIRF features (pcl library, opencv)
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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>

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

#include "registrorgb.h"
#include "registro3d.h"

//impostazioni utili al debug
#define DEBUG 0

using namespace std;
using namespace DBoW2;
using namespace DUtils;
using namespace cv;

typedef vector<vector<vector<float> > > BoWFeatures;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void searchRegistro();
void VocAuxNarf();
void VocAuxSurf();

void loopClosingRGB(BoWFeatures &features);
void loopClosing3d(BoWFeatures &features);

void loadFeatures3d(BoWFeatures &features);
void loadFeaturesRGB(BoWFeatures &features);
void testVocCreation(BoWFeatures &features,BoWFeatures &featuresrgb);
void listFile(string direc, vector<string> *files_lt);
void changeStructure(const vector<float> &plain, vector<vector<float> > &out,int L);
void readPoseFile(const char *filename,  vector<double> &xs,  vector<double> &ys);
int inliersRGB(int origine, int destinazione);
void wait();
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

vector<string> files_list_rgb,files_list_3d;
map<string, string> registro_interno;
map<int, string> registro_aux;
map<int, string> registro_aux_rgb;
//registri
map<int, string> registro_aux2;
map<int, string> registro_aux3;

typedef pair <string, string> PairString;
typedef pair <int, string> Mappa;
typedef pair <int, int> PivotMappa;

string filename_voc_3d = "voc_3d.yml.gz";
string filename_voc_rgb = "voc_rgb.yml.gz";

RegistroRGB* reg_RGB;
Registro3D* reg_3D;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
int main(int nNumberofArgs, char* argv[])
{
    
    string directory3d = "debug_pcd/";
    string directory3dbin = "pcdbin/";
    string directoryrgb = "_images/";
    string debug_directory3d = "debug_pcd/";
    string debug_directoryrgb = "debug_images/";
    
    BoWFeatures featuresrgb,features3d;
    bool flag_voc=false, flag_debug=false,flag_loop = false,flag_bin = false,flag_db=false;
    
    if(nNumberofArgs > 0){
        vector<string> parametri;
        for(int y = 0 ; y < nNumberofArgs; y++)
        {
            string parm = argv[y];
            StringFunctions::trim(parm);
            parametri.push_back(parm);
        }
        if ( find(parametri.begin(), parametri.end(), "-S") != parametri.end()){
            flag_db= true;
            cout << "--- OPZIONE SALVA DATABASE ---" << endl;
        }
        
        if ( find(parametri.begin(), parametri.end(), "-b") != parametri.end()){
            flag_bin= true;
            cout << "--- OPZIONE DEPTH IN FORMATO BINARIO ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "-l") != parametri.end()){
            flag_loop= true;
            cout << "--- OPZIONE LOOP CLOSING ---" << endl;
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
            cout << "[ -b ]: Si utilizzano il file depth in formato binario." << endl;
            cout << "[ -S ]: Salva DATABASE ed esce." << endl;
            cout << "[ -D ]: Modalità DEBUG. Dataset ridotto cartelle debug_{*}" << endl;
            exit(0);
        }else{
            if(flag_db){
                searchRegistro();
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
                loadFeaturesRGB(featuresrgb);
                loadFeatures3d(features3d);
                testVocCreation(features3d,featuresrgb);
                //salvaDB todo
                exit(0);
            }
            if (flag_voc || flag_loop){
                searchRegistro();
                
                if (flag_debug){
                    directory3d = debug_directory3d;
                    directoryrgb = debug_directoryrgb;
                }
                if(flag_bin){
                    directory3d = directory3dbin;
                }
                
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
                
                loadFeaturesRGB(featuresrgb);
                loadFeatures3d(features3d);
                VocAuxNarf();
                VocAuxSurf();
                testVocCreation(features3d,featuresrgb);
                if(flag_loop){
                    loopClosingRGB(featuresrgb);
                    //loopClosing3d(features3d);
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

void loopClosingRGB(BoWFeatures &features){
    
    int const INITIAL_OFFSET = 80; 
    double const MATCH_THRESHOLD = 0.4; 
    double const SANITY_THRESHOLD = 0.7;
    int const END_OFFSET = INITIAL_OFFSET;
    int const SIZE_BUCKET = 5; 
    int const TEMP_CONS = 4;
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
    
    Surf128Vocabulary voc_rgb(filename_voc_rgb);
    Surf128Database db_rgb(voc_rgb, false);
    
    vector<int> bucket;
    if (!DEBUG) { wait(); }
    
    for (int i=0;i<files_list_rgb.size();i++){
        loop_found = false;
        Mat im = imread(files_list_rgb[i]);
        DUtilsCV::GUI::showImage(im, true, &win, 10);
        if ((i+1)>INITIAL_OFFSET){
            
            BowVector v1, v2;
            voc_rgb.transform(features[i], v1);                
            voc_rgb.transform(features[i-1], v2);    
            if (voc_rgb.score(v1,v2) >= SANITY_THRESHOLD){           
                QueryResults ret;
                db_rgb.query(features[i], ret,db_rgb.size());
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
                            voc_rgb.transform(features[i-pivot], v1);                        
                            voc_rgb.transform(features[cursore], v2);
                            double score = voc_rgb.score(v1, v2);   
                            cout << (i-pivot) << " vs " << cursore << " score: " << score<<endl;
                            if (!(score >= MATCH_THRESHOLD)){               
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
        db_rgb.add(features[i]);       
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }
    db_rgb.save("DBRGB.yml.gz");    
}





void loopClosing3d(BoWFeatures &features)
{
    int const INITIAL_OFFSET = 80; 
    double const SANITY_THRESHOLD = 0.85;
    double const MATCH_THRESHOLD = 0.71;
    int const END_OFFSET = INITIAL_OFFSET; //elimino la coda, sicuramente darà buoni risultati e non è un loop
    int const SIZE_BUCKET = 5; //risultati da considerare validi e su cui determinare inliers
    int const TEMP_CONS = 4;
    bool loop_found = false;
    
    if (!DEBUG) { wait(); }
    //soglia 0.71 per 3D
    vector<int> bucket;
    vector<double> xs, ys;
    
    readPoseFile("g_truth_3d.txt", xs, ys);
    cout << "3D: Acquisizione Ground Truth" << endl;
    
    // prepare visualization windows
    
    DUtilsCV::GUI::tWinHandler winplot = "Traiettoria3D";
    
    DUtilsCV::Drawing::Plot::Style normal_style(2); // thickness
    DUtilsCV::Drawing::Plot::Style loop_style('r', 2); // color, thickness
    
    DUtilsCV::Drawing::Plot implot(240, 320,
                                   - *std::max_element(xs.begin(), xs.end()),
                                   - *std::min_element(xs.begin(), xs.end()),
                                   *std::min_element(ys.begin(), ys.end()),
                                   *std::max_element(ys.begin(), ys.end()), 25);
    NarfVocabulary voc_3d(filename_voc_3d);
    NarfDatabase db_3d(voc_3d, false);
    
    for (int i=0;i<files_list_3d.size();i++){
        loop_found = false;
        if ((i+1)>INITIAL_OFFSET){            
            BowVector v1, v2;
            voc_3d.transform(features[i], v1);                
            voc_3d.transform(features[i-1], v2);    
            if (voc_3d.score(v1,v2) >= SANITY_THRESHOLD){           
                QueryResults ret;
                db_3d.query(features[i], ret,db_3d.size());
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
                            voc_3d.transform(features[i-pivot], v1);                        
                            voc_3d.transform(features[cursore], v2);
                            double score = voc_3d.score(v1, v2);   
                            cout << (i-pivot) << " vs " << cursore << " score: " << score<<endl;
                            if (!(score >= MATCH_THRESHOLD)){               
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
                        tyu = reg_3D->getScoreFit(i,bucket[yy]);
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
        db_3d.add(features[i]);       
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }
    db_3d.save("DB3D.yml.gz");
}

void wait()
{
    cout << "Premi 'enter' per continuare." << endl;
    getchar();
}



void loadFeaturesRGB(BoWFeatures &features)
{
    features.clear();
    features.reserve(files_list_rgb.size());
    cv::SURF surf(300, 5, 4, true);
    
    string file_name_work,filename;
    ofstream save("vocsurf.aux");
    
    reg_RGB = new RegistroRGB(files_list_rgb.size());
    
    for (int i = 0; i < files_list_rgb.size(); ++i) {
        
        filename = files_list_rgb[i];
        cout << "Estrazione SURF per " << filename << endl;
        
        vector<string> vec1,vec2;
        StringFunctions::split(filename,vec1,"/");
        string w = vec1[vec1.size()-1];
        StringFunctions::trim(w);
        StringFunctions::split(w,vec2,".");
        file_name_work = vec2[0];
        
        cout << i<<") "<< file_name_work << " => " << registro_interno.find(file_name_work)->second <<endl;
        save << i << ":" << file_name_work << " => " <<registro_interno.find(file_name_work)->second << endl;
        registro_aux_rgb.insert(Mappa(i,file_name_work+" => "+registro_interno.find(file_name_work)->second));
        
        cv::Mat image = cv::imread(filename);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        vector<float> descriptors;
        surf(image, mask, keypoints, descriptors);
        features.push_back(vector<vector<float> >());
        reg_RGB->addFrame(filename,descriptors,keypoints);
        
        changeStructure(descriptors, features.back(), surf.descriptorSize());
        cout << "Estratti " << features[i].size() << " descrittori." << endl;
        cout << "-------------------------------------------------------" << endl;
        descriptors.clear();
        keypoints.clear();
    }
    cout << "Estrazione terminata." << endl;
}

// ----------------------------------------------------------------------------

void changeStructure(const vector<float> &plain, vector<vector<float> > &out,
                     int L)
{
    out.resize(plain.size() / L);
    int j = 0;
    for(int i = 0; i < plain.size(); i += L, ++j)
    {
        out[j].resize(L);
        copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
    }
}

// ----------------------------------------------------------------------------
void loadFeatures3d(BoWFeatures &features)
{
    typedef pcl::PointXYZ PointType;
    float angular_resolution = pcl::deg2rad (0.3);
    float support_size = 0.1f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    
    reg_3D = new Registro3D("pcd/");
    
    features.clear();
    features.reserve(files_list_3d.size());
    
    string filename;
    string file_name_work;
    ofstream save("voc.aux");
    
    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 1;
    
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    
    for (int i = 0; i < files_list_3d.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr point_cloud_wf (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
        
        filename = files_list_3d[i];
        pcl::io::loadPCDFile (filename, *point_cloud_wf);
        
        //filtraggio valori NaN
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud (*point_cloud_wf,*point_cloud_wf,indices);
        //filtraggio con pixel volumetrici
        const float VOXEL_GRID_SIZE = 0.01f;
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
        vox_grid.setInputCloud( point_cloud_wf );
        vox_grid.filter( *point_cloud );
        
        cout << "Estrazione NARF per " << filename << endl;
        
        Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
        //SCENE_SENSOR_POSE
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f ((*point_cloud).sensor_origin_[0],
                                             (*point_cloud).sensor_origin_[1],
                (*point_cloud).sensor_origin_[2])) *
                Eigen::Affine3f ((*point_cloud).sensor_orientation_);
        
        //ricavo il nome file e lo registro
        vector<string> vec1,vec2;
        StringFunctions::split(filename,vec1,"/");
        string w = vec1[vec1.size()-1];
        StringFunctions::trim(w);
        StringFunctions::split(w,vec2,".");
        file_name_work = vec2[0];
        cout << i<<") "<< file_name_work << " => " << registro_interno.find(file_name_work)->second <<endl;
        save << i << ":" << file_name_work << " => " <<registro_interno.find(file_name_work)->second << endl;
        registro_aux.insert(Mappa(i,file_name_work+" => "+registro_interno.find(file_name_work)->second));
        
        vec1.clear();
        vec2.clear();
        
        range_image.createFromPointCloud (*point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                          scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
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
        
        // ------------------------------------------------------
        // -----Calcolo NARF descriptors per i punti di interesse-----
        // ------------------------------------------------------
        vector<int> keypoint_indices2;
        keypoint_indices2.resize (keypoint_indices.points.size ());
        for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
            keypoint_indices2[i]=keypoint_indices.points[i];
        pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
        narf_descriptor.getParameters().support_size = support_size;
        narf_descriptor.getParameters().rotation_invariant = true;
        pcl::PointCloud<pcl::Narf36> narf_descriptors;
        narf_descriptor.compute (narf_descriptors);
        cout << "Estratti "<<narf_descriptors.size ()<<" descrittori per "
             <<keypoint_indices.points.size ()<< " keypoints.\n";
        features.push_back(vector<vector<float> >());
        cout << "------------------------------------------------------------" << endl;
        
        for (int p = 0; p < narf_descriptors.size(); p++) {
            vector<float> flot;
            copy(narf_descriptors[p].descriptor, narf_descriptors[p].descriptor+FNarf::L, back_inserter(flot));
            features.back().push_back(flot);
            flot.clear();
        }
        range_image.clear();
        (*point_cloud).clear();
        (*point_cloud_wf).clear();
        narf_descriptors.clear();
        narf_descriptor = NULL;
    }
    cout << "Estrazione terminata." << endl;
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

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------
void VocAuxSurf()
{
    string line;
    ifstream Myfile;
    Myfile.open("vocsurf.aux");
    
    if(Myfile.is_open()) {
        while(Myfile.peek() != EOF) {
            vector<string> w1;
            getline(Myfile, line);
            StringFunctions::trim(line);
            if(line.size() > 0) {
                //luogo
                StringFunctions::split(line,w1,":");
                string w = w1[1];
                StringFunctions::trim(w);
                string h = w1[0];
                StringFunctions::trim(h);
                registro_aux3.insert(Mappa(atoi(h.c_str()),w));
                
            }
        }
        Myfile.close();
    } else {
        cout<<"vocsurf.aux NON TROVATO."<<endl;
    }
}

void VocAuxNarf()
{
    string line;
    ifstream Myfile;
    Myfile.open("voc.aux");
    
    if(Myfile.is_open()) {
        while(Myfile.peek() != EOF) {
            vector<string> w1;
            getline(Myfile, line);
            StringFunctions::trim(line);
            if(line.size() > 0) {
                //luogo 
                StringFunctions::split(line,w1,":");
                string w = w1[1];
                StringFunctions::trim(w);
                string h = w1[0];
                StringFunctions::trim(h);
                registro_aux2.insert(Mappa(atoi(h.c_str()),w));
                
            }
        }
        Myfile.close();
    } else {
        cout<<"voc.aux NON TROVATO."<<endl;
    }
}
