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
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>

// DBoW2
#include "DBoW2.h"

#include "DUtils/DUtils.h"
#include "DUtilsCV/DUtilsCV.h"
#include "DVision/DVision.h"
#include "TwoWayMatcher.h"
#include "registrorgb.h"

using namespace std;
using namespace DBoW2;
using namespace DUtils;
using namespace cv;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void searchRegistro();
void VocAux();
void VocAux2();
void loopClosing(const vector<vector<vector<float> > > &features,const vector<vector<vector<float> > > &features2);
void loadFeatures(vector<vector<vector<float> > > &features);
void loadFeaturesRGB(vector<vector<vector<float> > > &features);
void writeFeatures(string nomefile, vector < vector < vector <float > > > &feat);
void testVocCreation(const vector<vector<vector<float> > > &features,const vector<vector<vector<float> > > &featuresrgb);
void listFile(string direc, vector<string> *files_lt);
void changeStructure(const vector<float> &plain, vector<vector<float> > &out,int L);
void readPoseFile(const char *filename,  vector<double> &xs,  vector<double> &ys);
int inliersRGB(cv::Mat descriptors1,cv::Mat descriptors2,vector<cv::KeyPoint> keypoints1,vector<cv::KeyPoint> keypoints2);
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

vector<CoppiaRGB*> reg_RGB;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int main(int nNumberofArgs, char* argv[])
{

    string directory3d = "pcd/";
    string directoryrgb = "_images/";

    string debug_directory3d = "debug_pcd/";
    string debug_directoryrgb = "debug_images/";

    vector<vector<vector<float> > > featuresrgb,features3d;
    bool flag_voc=false, flag_debug=false,flag_loop = false;

    if(nNumberofArgs > 0){
        vector<string> parametri;
        for(int y = 0 ; y < nNumberofArgs; y++)
        {
            string parm = argv[y];
            StringFunctions::trim(parm);
            parametri.push_back(parm);
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
            cout << "[ -D ]: Modalità DEBUG. Usa un dataset ridotto nelle cartelle debug_{*}" << endl;
            exit(0);
        }else{
            if (flag_voc || flag_loop){
                searchRegistro();

                if (flag_debug){
                    directory3d = debug_directory3d;
                    directoryrgb = debug_directoryrgb;
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
                loadFeatures(features3d);
                VocAux();
                VocAux2();
                testVocCreation(features3d,featuresrgb);
                if(flag_loop){
                    loopClosing(features3d,featuresrgb);

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

void loopClosing(const vector<vector<vector<float> > > &features,const vector<vector<vector<float> > > &features2)
{

    int const INITIAL_OFFSET = 70;
    double const MATCH_THRESHOLD = 0.40;
    int const END_OFFSET = INITIAL_OFFSET; //elimino la coda, sicuramente darà buoni risultati e non è un loop
    int const GOOD_SUBSET = 4; //risultati da considerare validi e su cui determinare inliers
    int const TEMP_CONS = 3;
    bool loop_found = false;
    // load robot poses
    vector<double> xs, ys;
    readPoseFile("g_truth_rgb.txt", xs, ys);
    cout << "Acquisizione Ground Truth" << endl;
    cout << "xs: " << xs.size() << endl;
    cout << "ys: " << ys.size() << endl;
    // prepare visualization windows
    DUtilsCV::GUI::tWinHandler win = "Immagine Analizzata";
    DUtilsCV::GUI::tWinHandler winplot = "TraiettoriaRGB";

    DUtilsCV::Drawing::Plot::Style normal_style('b',1); // thickness
    DUtilsCV::Drawing::Plot::Style loop_style('r', 2); // color, thickness

    DUtilsCV::Drawing::Plot implot(240, 320,
                                   - *std::max_element(xs.begin(), xs.end()),
                                   - *std::min_element(xs.begin(), xs.end()),
                                   *std::min_element(ys.begin(), ys.end()),
                                   *std::max_element(ys.begin(), ys.end()), 25);

    Surf128Vocabulary voc_rgb(filename_voc_rgb);
    Surf128Database db_rgb(voc_rgb, false);

    vector<int> goodSubset;
    int TMP_INITIAL_OFFSET = INITIAL_OFFSET;

    wait();

    for (int i=0;i<files_list_rgb.size();i++){
        loop_found = false;
        Mat im = imread(files_list_rgb[i]);
        DUtilsCV::GUI::showImage(im, true, &win, 10);
        if ((i+1)>TMP_INITIAL_OFFSET){
            QueryResults ret;
            db_rgb.query(features2[i], ret,db_rgb.size());
            CoppiaRGB* src = reg_RGB[i];
            for (int j = 0; j < ret.size(); j++){ //scansiono risultati
                if (ret[j].Score > MATCH_THRESHOLD){ //sanity check
                    if ((i - END_OFFSET) >= ret[j].Id){ //scarto la coda
                        cout << i << " vs " << ret[j].Id << " score: " << ret[j].Score << endl;
                        if (goodSubset.size() <= GOOD_SUBSET){
                            goodSubset.push_back(ret[j].Id);
                            cout << i << " in goodSubset : " << ret[j].Id << endl;
                            cout << i << " vs "  << ret[j].Id << " -> "<<ret[j].Score << endl;
                            cout << "----------------------------" << endl;
                        }
                        if (goodSubset.size() == GOOD_SUBSET){
                            int maxInliers = 0;
                            int maxIdInliers = -1;
                            int tyu = 0;
                            for(int yy = 0; yy < goodSubset.size(); yy++){
                                CoppiaRGB* dst = reg_RGB[goodSubset[yy]];
                                cout << "key: " << dst->keypoints.size() << " good: " << goodSubset[yy] << endl;
                                tyu = inliersRGB((Mat)src->descriptors, (Mat)dst->descriptors,src->keypoints,dst->keypoints);
                                if (maxInliers < tyu){ //trovo il massimo del subset buono.
                                    maxInliers = tyu;
                                    maxIdInliers = goodSubset[yy];
                                }
                            }
                            //consistenza temporale
                            BowVector v1, v2;

                            voc_rgb.transform(features[ret[j].Id], v1);
                            for(int yy = 0; yy < TEMP_CONS; yy++)
                            {
                                voc_rgb.transform(features[maxIdInliers - yy], v2);
                                double score = voc_rgb.score(v1, v2);
                                if (score >= MATCH_THRESHOLD){
                                    cout << "Image " << i << " vs Image " << maxIdInliers - yy << ": " << score << endl;
                                }else{
                                    break;
                                }
                            }
                            cout << "LOOP TROVATO - INLIERS MAX:" << tyu << ". "<< i <<" per immagine: " << maxIdInliers << endl;
                            TMP_INITIAL_OFFSET = INITIAL_OFFSET + i; // se individuo un loop devo ripartire.
                            implot.line(-xs[maxIdInliers], ys[maxIdInliers], -xs[i], ys[i], loop_style);
                            j = ret.size();
                        }
                    }
                }
            }
            goodSubset.clear();
            ret.clear();
        }
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
        db_rgb.add(features2[i]);
    }
    db_rgb.clear();

    wait();

    //    NarfVocabulary voc_3d(filename_voc_3d);
    //    NarfDatabase db_3d(voc_3d, false);

    //db_3d.clear();
}

void writeFeatures(string nomefile, vector < vector < vector <float > > > &feat)
{
    ofstream f_txt(nomefile.c_str());
    for(int uu1 = 0; uu1 < feat.size(); uu1++)
    {
        f_txt << uu1 << "|";
        for(int uu2 = 0; uu2 < feat[uu1].size(); uu2++)
        {
            f_txt << uu2 << "|";
            for(int uu3 = 0; uu3 < feat[uu1][uu2].size(); uu3++)
            {
                if (uu3 == (feat[uu1][uu2].size()-1) )
                {
                    f_txt << boost::lexical_cast<string>(feat[uu1][uu2][uu3]);
                }
                else
                {
                    f_txt << boost::lexical_cast< string>(feat[uu1][uu2][uu3]) << "|";
                }
            }
        }
        f_txt << "#";
    }
    f_txt.close();
    //f = boost::lexical_cast<float>(s);
}


void wait()
{
    cout << "Premi 'enter' per continuare." << endl;
    getchar();
}

int inliersRGB(cv::Mat descriptors1,cv::Mat descriptors2,vector<cv::KeyPoint> keypoints1,vector<cv::KeyPoint> keypoints2)
{
    int numInliers = 0;
    TwoWayMatcher matcher(TWM_FLANN);
    //TwoWayMatcher matcher(TWM_BRUTEFORCE_L1);
    vector<cv::DMatch> matches;
    cv::Mat mask1, mask2;
    matcher.match(descriptors1, descriptors2, matches, mask1, mask2);

    // Initializes points lists
    vector<cv::Point2f > p1vec(matches.size());
    vector<cv::Point2f > p2vec(matches.size());
    Mat points1(p1vec);
    Mat points2(p2vec);

    vector<DMatch>::iterator matchIt;
    Mat_<Point2f>::iterator p1It = points1.begin<Point2f > ();
    Mat_<Point2f>::iterator p2It = points2.begin<Point2f > ();
    for (matchIt = matches.begin(); matchIt != matches.end(); matchIt++)
    {
        try{
            (*p1It).x = keypoints1[(*matchIt).queryIdx].pt.x;
            (*p1It).y = keypoints1[(*matchIt).queryIdx].pt.y;
            (*p2It).x = keypoints2[(*matchIt).trainIdx].pt.x;
            (*p2It).y = keypoints2[(*matchIt).trainIdx].pt.y;
            p1It++;
            p2It++;
        }
        catch (std::exception& e)
        {
            std::cerr << "exception caught: " << e.what() << '\n';
        }
    }
    vector<uchar> inliersMask;
    //if(points1.cols>4 && points1.rows>4 && points2.cols>4 && points2.rows>4){
    findHomography(Mat(points1), Mat(points2), inliersMask, CV_FM_RANSAC, 1);
    numInliers =  count(inliersMask.begin(), inliersMask.end(), 1);
    //}
    points1.empty();
    points2.empty();
    p1vec.clear();
    p2vec.clear();

    return numInliers;
}

void loadFeaturesRGB(vector<vector<vector<float> > > &features)
{
    features.clear();
    features.reserve(files_list_rgb.size());
    cv::SURF surf(300, 5, 4, true);

    string file_name_work,filename;
    ofstream save("vocsurf.aux");

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

        reg_RGB.push_back(new CoppiaRGB(filename,descriptors,keypoints));

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
typedef pcl::PointXYZ PointType;
float angular_resolution = pcl::deg2rad (0.5); //0.08
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

void loadFeatures(vector<vector<vector<float> > > &features)
{
    features.clear();
    features.reserve(files_list_3d.size());

    string filename;
    string file_name_work;
    ofstream save("voc.aux");

    for (int i = 0; i < files_list_3d.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;

        filename = files_list_3d[i];
        pcl::io::loadPCDFile (filename, point_cloud);

        cout << "Estrazione NARF per " << filename << endl;

        Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity());
        //SCENE_SENSOR_POSE
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                                   point_cloud.sensor_origin_[1],
                                                                   point_cloud.sensor_origin_[2])) *
                Eigen::Affine3f (point_cloud.sensor_orientation_);

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



        // -----------------------------------------------
        // -----Crea RangeImage dal PointCloud-----
        // -----------------------------------------------
        float noise_level = 0.0f;
        float min_range = 0.0f;
        int border_size = 1;
        boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
        pcl::RangeImage& range_image = *range_image_ptr;
        range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                          scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
        //range_image.integrateFarRanges (far_ranges);
        range_image.setUnseenToMaxRange();
        point_cloud.clear();
        // --------------------------------
        // -----Estraggo NARF keypoints----
        // --------------------------------
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

        range_image.clear();
        keypoint_indices.clear();
        keypoint_indices2.clear();

        for (unsigned int p = 0; p < narf_descriptors.size(); p++) {
            vector<float> flot;
            copy(narf_descriptors[p].descriptor, narf_descriptors[p].descriptor+FNarf::L, back_inserter(flot));
            features.back().push_back(flot);
            flot.clear();
        }
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

void testVocCreation(const vector<vector<vector<float> > > &features,const vector<vector<vector<float> > > &featuresrgb)
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

    BowVector v1, v2;
    ofstream bown("bow.txt");
    const static int taglia = (pow(k,L)*2) + 1; //il +1 si riferisce all'etichetta del luogo.
    const static int offset = taglia/2;

    for(int i ; i < files_list_3d.size(); i++)
    {
        voc.transform(features[i], v1);
        voc2.transform(featuresrgb[i], v2);

        double bbow[taglia];
        for(int jay = 0; jay < taglia;jay++){
            bbow[jay] = 0;
        }
        //RGB
        for(BowVector::iterator vit = v2.begin(); vit != v2.end(); vit++){
            int pivot = vit->first;
            bbow[pivot] = vit->second;
        }
        //3D
        for(BowVector::iterator vit = v1.begin(); vit != v1.end(); vit++){
            int pivot = offset + vit->first;
            bbow[pivot] = vit->second;
        }

        //inserimento etichetta luogo per generare il file che verrà inviato all'apprendimento.
        string luogo = registro_aux.at(i);
        vector<string> a;
        boost::split(a, registro_aux.at(i), boost::is_any_of("=>"));
        a.erase(  remove_if( a.begin(), a.end(), boost::bind( & string::empty, _1 ) ), a.end());
        StringFunctions::trim(a[1]);
        bown << a[1] << ",";

        for(int jay = 0; jay < taglia;jay++){
            bown << bbow[jay];
            if (jay < taglia - 1){ bown << ","; }
        }
        bown << endl;
    }
    bown.close();
}

// ----------------------------------------------------------------------------
void VocAux2()
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

void VocAux()
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
