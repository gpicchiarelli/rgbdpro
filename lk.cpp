/**
 * File: lk.cpp
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

#include "lk.h"

//impostazioni utili al debug
#define DEBUG 0

RegistroRGB* reg_RGB;
Registro3D* reg_3D;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
string directory3d = "pointscloud/";
string directory3dbin = "pcdbin/";
string directoryrgb = "rgb/";
string debug_directory3d = "debug_pcd/";
string debug_directoryrgb = "debug_images/";
string datasets_root = "Dataset/";
string filename_voc_3d,filename_voc_rgb,dataset_sub;

double SOGLIA_3D = 0.71;
double SOGLIA_RGB = 0.45;
int I_OFFSET = 80;
int VOC = 3;
double SANITY = 0.2;

#include <omp.h>

int main(int nNumberofArgs, char* argv[])
{    
    BoWFeatures featuresrgb,features3d;
    bool flag_voc=false, flag_debug=false,flag_loop = false,flag_bin = false,flag_s=false,flag_u=false;
    bool flag_l3d=false,flag_lrgb=false,flag_stats=false,flag_s3d=false;
    if(nNumberofArgs > 0){
        vector<string> parametri;
        bool dataset_splu = false;
        for(int y = 0 ; y < nNumberofArgs; y++)
        {
            string parm = argv[y];
            StringFunctions::trim(parm);
            parametri.push_back(parm);  
            if (boost::starts_with(parm,"--s-3d=")){   
                string h = argv[y];
                boost::erase_all(h, "--s-3d="); 
                SOGLIA_3D = lexical_cast<double>(h);            
            }
            if (boost::starts_with(parm,"--s-rgb=")){   
                string h = argv[y];
                boost::erase_all(h, "--s-rgb="); 
                SOGLIA_RGB = lexical_cast<double>(h);
            }
            if (boost::starts_with(parm,"--offset=")){   
                string h = argv[y];
                boost::erase_all(h, "--offset="); 
                I_OFFSET = lexical_cast<int>(h);
            }
            if (boost::starts_with(parm,"--sanity=")){   
                string h = argv[y];
                boost::erase_all(h, "--sanity="); 
                SANITY = lexical_cast<double>(h);
            }
            if (boost::starts_with(parm,"--voc=")){   
                string h = argv[y];
                boost::erase_all(h, "--voc="); 
                VOC = lexical_cast<int>(h);
            }
            if (boost::starts_with(parm,"--root-datasets=")){   
                string h = argv[y];
                boost::erase_all(h, "--root-datasets=");               
                datasets_root = h+"/";
            }
            if (boost::starts_with(parm,"--dataset=")){   
                string h = argv[y];
                boost::erase_all(h, "--dataset=");               
                dataset_splu = true;
                directory3d = datasets_root+h+"/"+directory3d;
                directory3dbin = datasets_root+h+"/"+directory3dbin;                
                directoryrgb = datasets_root+h+"/"+directoryrgb;
                dataset_sub = datasets_root+h+"/";
            }
        }
        
        if(!dataset_splu && find(parametri.begin(), parametri.end(), "-h") == parametri.end() &&
                find(parametri.begin(), parametri.end(), "--stats") == parametri.end()){
            cout << "ERRORE: Fornire un dataset --dataset=NOMESOTTOCARTELLA oppure inserire '-h'";
            return 1;
        }
        filename_voc_3d = dataset_sub+"voc_3d.yml.gz";
        filename_voc_rgb = dataset_sub+"voc_rgb.yml.gz";
        if ( find(parametri.begin(), parametri.end(), "-U") != parametri.end()){
            flag_u= true;
            cout << "--- OPZIONE UTILIZZA FEATURES SALVATE ---" << endl;
        }
        if ( find(parametri.begin(), parametri.end(), "--stats") != parametri.end()){
            flag_stats= true;
        }
        if ( find(parametri.begin(), parametri.end(), "-S") != parametri.end()){
            flag_s= true;
            cout << "--- OPZIONE SALVA FEATURES IN FORMATO BINARIO ---" << endl;
        }     
        if ( find(parametri.begin(), parametri.end(), "-S-3d") != parametri.end()){
            flag_s3d= true;
            cout << "--- OPZIONE SALVA FEATURES 3D IN FORMATO BINARIO ---" << endl;
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
            cout << endl << "DBOW NARF+SURF128 test: ";
            cout << endl;
            cout <<"Nome Vocabolario NARF: "<< filename_voc_3d<< endl;
            cout <<"Nome Vocabolario SURF128: "<< filename_voc_rgb<< endl;
            cout << endl;
            cout <<"ground truth NARF: syncPCDGT.txt"<<endl;
            cout <<"ground truth SURF128: syncRGBGT.txt "<< endl;
            cout << endl;
            cout << "[ -h ]: Guida" << endl;
            cout << "[ -vdel ]: Cancella i vecchi vocabolari." << endl;
            cout << "[ -l ]: Loop Closing." << endl;
            cout << "[ -l-rgb ]: Loop Closing RGB." << endl;
            cout << "[ -l-3d ]: Loop Closing 3D." << endl;
            cout << "[ -b ]: Si utilizzano il file depth in formato binario." << endl;
            cout << "[ -S ]: Salva DATABASE ed esce." << endl;
            cout << "[ -D ]: Modalità DEBUG. Dataset ridotto cartelle debug_{*}" << endl;
            cout << "[ --dataset=NOMEDATASET ]: Sottocartella di Dataset. Specifica un dataset." << endl;
            cout << "[ --root-datasets=CARTELLADATASETS ]: Specifica una directory di datasets. DEFAULT=Datasets" << endl;  
            cout << "[ --s-rgb=sogliargb ]: soglia per il match rgb" << endl;           
            cout << "[ --s-3d=soglia3d ]: soglia per il match 3d" << endl;    
            cout << "[ --offset=offset ]: offset" << endl; 
            cout << "[ --sanity=sanity ]: match per sanity check" << endl; 
            cout << "[ --stats ]: Valuta solo statistiche, se presenti file di report" << endl;           
            
            exit(0);
        }else{
            if(flag_stats){
                try{
                    searchRegistro(dataset_sub+"registro.txt");
                    cout << "File registro: " <<registro_interno.size()<< " elementi."<<endl;
                }catch(std::exception& e){
                    cout << "errore: searchRegistro()"	<<endl;
                }
                stats* owl = new stats("report_3d.rep",registro_interno,I_OFFSET,"3D");
                cout << owl->toString();
                stats* owl2 = new stats("report_rgb.rep",registro_interno,I_OFFSET,"RGB");
                cout << owl2->toString();
            }
            
            if (flag_voc || flag_loop || flag_s  || flag_s3d || flag_u){
                try{
                    searchRegistro(dataset_sub+"registro.txt");
                    cout << "File registro: " <<registro_interno.size()<< " elementi."<<endl;
                }catch(std::exception& e){
                    cout << "errore: searchRegistro()"	<<endl;
                    exit(1);
                }
                
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
                    loadFeaturesFile(features3d,dataset_sub+"feat_3d.dat");
                    loadFeaturesFile(featuresrgb,dataset_sub+"feat_rgb.dat");
                }else{
                    if(!flag_s && !flag_s3d){
                        loadFeaturesRGB(featuresrgb);
                        loadFeatures3d(features3d);
                    }
                }
                if(flag_s){                        
                    loadFeaturesRGB(featuresrgb);
                    saveFeaturesFile(featuresrgb,dataset_sub+"feat_rgb.dat");
                    featuresrgb.clear();
                    
                    loadFeatures3d(features3d);
                    saveFeaturesFile(features3d,dataset_sub+"feat_3d.dat");
                    features3d.clear();
                    
                    loadFeaturesFile(features3d,dataset_sub+"feat_3d.dat");
                    loadFeaturesFile(featuresrgb,dataset_sub+"feat_rgb.dat");
                    testVocCreation(features3d,featuresrgb);
                    return 0;
                }
                if(flag_s3d){
                    loadFeatures3d(features3d);
                    saveFeaturesFile(features3d,dataset_sub+"feat_3d.dat");
                    features3d.clear();
                    exit(0);
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

void loopClosingRGB(const BoWFeatures &features){
    
    int OFFSET = I_OFFSET;
    double MATCH_THRESHOLD = SOGLIA_RGB;
    double SANITY_THRESHOLD = SANITY;
    int SIZE_BUCKET = 5;
    int TEMP_CONS = 4;
    bool loop_found = false;
    
    // load robot poses
    vector<double> xs, ys;
    readPoseFile((dataset_sub+"syncRGBGT.txt").c_str(), xs, ys);
    cout << "RGB: Acquisizione Ground Truth" << endl;
    
    ofstream report("report_rgb.rep");
    
    
    // prepare visualization windows
    DUtilsCV::GUI::tWinHandler win = "Immagine Analizzata";
    DUtilsCV::GUI::tWinHandler winplot = "TraiettoriaRGB";
    
    DUtilsCV::Drawing::Plot::Style normal_style(1); // thickness
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
        if ((i+1)>OFFSET){
            BowVector v1, v2;
            voc.transform(features[i], v1);
            voc.transform(features[i-1], v2);
            double san =  voc.score(v1,v2);
            cout << "S:" << san << endl;
            if (san >= SANITY_THRESHOLD){
                QueryResults ret;
                db.query(features[i], ret,db.size());
                for (int j = 0; j < ret.size(); j++){
                    if (ret[j].Score > MATCH_THRESHOLD){
                        if ((i - OFFSET) >= ret[j].Id){
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
                    vector<int> i_back; //contiene le precedenti
                    
                    for(int yyy = 0; yyy < TEMP_CONS; yyy++){
                        i_back.push_back((i-(yyy+1)));
                    }                    
                    BowVector v1, v2;
                    int centro;
                    for (int aa = 0 ; aa < bucket.size(); aa++){//per ogni elemento del bucket        
                        
                        double max_score = 0;
                        int max_id = -1;                        
                        for (int cc = 0; cc<i_back.size();cc++){                            
                            voc.transform(features[i - (cc+1)],v1);                            
                            if (max_id == -1 || centro == -1){ centro = bucket[aa]; }                           
                            
                            for(int bb = (centro - (TEMP_CONS/2) - 1) ; bb < centro + (TEMP_CONS/2); bb++){
                                int cursore = bb + 1;
                                if (cursore < 0) {max_id = -1; continue;}
                                
                                voc.transform(features[cursore],v2);
                                double score = voc.score(v1,v2);
                                cout << i - (cc+1) << " vs " << cursore << " score: " << score << endl;
                                if (score >= MATCH_THRESHOLD && score > max_score){
                                    max_id = cursore;
                                    max_score = score;
                                }                                
                            }
                            if (max_id == -1){
                                bucket[aa] = -1;
                                centro = -1;
                                max_score = 0;
                                break;
                            }else{
                                centro = max_id;
                                max_score = 0;
                            }
                        }
                    }
                }
                bucket.erase(std::remove( bucket.begin(), bucket.end(),-1), bucket.end()); 
                int maxInliers = 0;
                int maxIdInliers = -1;
                int tyu = 0;
                if(bucket.size() == 1){
                    maxIdInliers = bucket[0];
                }else{                    
                    for(int yy = 0; yy < bucket.size(); yy++){                        
                        tyu = reg_RGB->inliersRGB(i,bucket[yy]);
                        cout << "Inliers: " << tyu << " per immagine " << bucket[yy] << endl;
                        if (maxInliers < tyu){
                            maxInliers = tyu;
                            maxIdInliers = bucket[yy];
                        }                        
                    }                
                }
                if(maxIdInliers != -1){
                    loop_found = true;
                    cout << "LOOP TROVATO : " << i <<" per immagine: " << maxIdInliers << endl;
                    implot.line(-xs[maxIdInliers], ys[maxIdInliers], -xs[i], ys[i], loop_style);
                }
                bucket.clear();
                if(loop_found)
                {
                    string ii,maxid;
                    ii = boost::lexical_cast<string>(i);
                    maxid = boost::lexical_cast<string>(maxIdInliers);
                    report << i <<":"<< maxIdInliers <<endl;
                }
            }
        }
        db.add(features[i]);
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }
    report.close();
    stats* owl = new stats("report_rgb.rep",registro_interno,OFFSET,"RGB");
    cout << owl->toString();
}


void loopClosing3d(const BoWFeatures &features)
{    
    int OFFSET = I_OFFSET;
    double SANITY_THRESHOLD = SANITY;
    double MATCH_THRESHOLD = SOGLIA_3D;
    int SIZE_BUCKET = 5;
    int TEMP_CONS = 4;
    bool loop_found = false;
    
    if (!DEBUG) { wait(); }
    vector<int> bucket;
    vector<double> xs, ys;
    
    readPoseFile((dataset_sub+"syncPCDGT.txt").c_str(), xs, ys);
    cout << "3D: Acquisizione Ground Truth" << endl;
    ofstream report("report_3d.rep");
    
    DUtilsCV::GUI::tWinHandler winplot = "Traiettoria3D";
    
    DUtilsCV::Drawing::Plot::Style normal_style(1); // thickness
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
        if ((i+1)>OFFSET){
            BowVector v1, v2;
            voc.transform(features[i], v1);
            voc.transform(features[i-1], v2);
            double san =  voc.score(v1,v2);
            cout << "S:" << san << endl;
            if (san >= SANITY_THRESHOLD){
                QueryResults ret;
                db.query(features[i], ret,db.size());
                for (int j = 0; j < ret.size(); j++){ //scansiono risultati
                    if (ret[j].Score > MATCH_THRESHOLD){ //sanity check
                        if ((i - OFFSET) >= ret[j].Id){ //scarto la coda
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
                    vector<int> i_back; //contiene le precedenti
                    
                    for(int yyy = 0; yyy < TEMP_CONS; yyy++){
                        i_back.push_back((i-(yyy+1)));
                    }                    
                    BowVector v1, v2;
                    int centro;
                    for (int aa = 0 ; aa < bucket.size(); aa++){//per ogni elemento del bucket        
                        
                        double max_score = 0;
                        int max_id = -1;     
                        
                        
                        for (int cc = 0; cc<i_back.size();cc++){                            
                            voc.transform(features[i - (cc+1)],v1);                            
                            if (max_id == -1 || centro == -1){ centro = bucket[aa]; }                           
                            
                            for(int bb = (centro - (TEMP_CONS/2) - 1) ; bb < centro + (TEMP_CONS/2); bb++){
                                int cursore = bb + 1;
                                if (cursore < 0) {max_id = -1; continue;}
                                voc.transform(features[cursore],v2);
                                double score = voc.score(v1,v2);
                                cout << i - (cc+1) << " vs " << cursore << " score: " << score << endl;
                                if (score >= MATCH_THRESHOLD && score > max_score){
                                    max_id = cursore;
                                    max_score = score;
                                }                                
                            }
                            if (max_id == -1){
                                bucket[aa] = -1; 
                                centro = -1;
                                max_score = 0;
                                break;
                            }else{
                                centro = max_id;
                                max_score = 0;
                            }
                        }
                    }
                }
                bucket.erase(std::remove( bucket.begin(), bucket.end(),-1), bucket.end());
                double maxInliers = 0;//maxInliers = numeric_limits<double>::max();
                double maxIdInliers = -1;
                double tyu = 0;
                if(bucket.size() == 1){
                    maxIdInliers = bucket[0];
                }else{                    
                    for(int yy = 0; yy < bucket.size(); yy++){  
                        int term = bucket[yy];
                        tyu = reg_3D->getScoreFix(features[i],features[term]);//reg_3D->getScoreFit(i,bucket[yy]);
                        cout << "score: " << tyu <<endl;
                        if (maxInliers > tyu){
                            maxInliers = tyu;
                            maxIdInliers = bucket[yy];
                        }                        
                    }                
                }
                if(maxIdInliers != -1){
                    loop_found = true;
                    cout << "LOOP TROVATO : " << i <<" per immagine: " << maxIdInliers << endl;
                    implot.line(-xs[maxIdInliers], ys[maxIdInliers], -xs[i], ys[i], loop_style);
                }
                bucket.clear();
                if(loop_found)
                {
                    string ii,maxid;
                    ii = boost::lexical_cast<string>(i);
                    maxid = boost::lexical_cast<string>(maxIdInliers);
                    report << i <<":"<< maxIdInliers<<endl;
                    report.flush();
                }
            }
        }
        db.add(features[i]);
        implot.line(-xs[i-1], ys[i-1], -xs[i], ys[i], normal_style);
        DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);        
    }
    report.close();
    stats* owl = new stats("report_3d.rep",registro_interno,OFFSET,"3D");
    cout << owl->toString();
}

void loadFeaturesRGB(BoWFeatures &features)
{
    features.clear();
    features.reserve(files_list_rgb.size());
    cv::SURF surf(300, 4, 3, true);
    
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

//extract indices of largest planar component
pcl::PointIndices::Ptr extractIndicesPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_t){ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>());
    (*pcd) = (*pcd_t); //devo fare una copia altrimenti filtro l'originale
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (500);
    seg.setDistanceThreshold (0.01);
    
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (pcd);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // Extract the inliers
    extract.setInputCloud (pcd);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*pcd);
    
    return inliers;
}

// ----------------------------------------------------------------------------
void loadFeatures3d(BoWFeatures &features)
{
    typedef pcl::PointXYZ PointType;
    float angular_resolution = pcl::deg2rad (0.3f);
    float support_size = 0.1f;

    features.clear();
    features.reserve(files_list_3d.size());
    
    float noise_level = 0.0f;
    float min_range = 0.0f;
    int border_size = 1;
    
    //pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;    
    
    for (int i = 0; i < files_list_3d.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr point_cloud_wf (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
        
        pcl::io::loadPCDFile (files_list_3d[i], *point_cloud_wf);
        
        //filtraggio valori NaN
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud (*point_cloud_wf,*point_cloud_wf,indices);        
        
        const float VOXEL_GRID_SIZE = 0.01f;
        pcl::VoxelGrid<PointType> vox_grid;
        vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
        vox_grid.setInputCloud(point_cloud_wf);
        vox_grid.filter(*point_cloud);     
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (point_cloud);
        sor.setMeanK (60);
        sor.setStddevMulThresh (1.0);
        sor.filter (*point_cloud);
        
        cout << "Estrazione NARF: " << files_list_3d[i] ;
        
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

void testVocCreation(const BoWFeatures &features,const BoWFeatures &featuresrgb)
{ 
    // branching factor and depth levels
    const int k = 10;
    const int L = VOC;
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
        BoWFeatures tmp,tmp2; 
        for(int yy=0; yy<features.size(); yy+=5){
            tmp.push_back(features[yy]);
        }
        for(int yy=0; yy<featuresrgb.size(); yy+=5){
            tmp2.push_back(featuresrgb[yy]);
        }
        voc.create(tmp);
        Surf128Vocabulary voc2(k, L, weight, score);
        voc2.create(tmp2);
        
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
