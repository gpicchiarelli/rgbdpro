/**
 * File: stats.cpp
 * Date: April 2013
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

#include "stats.h"

struct ordering_factory_int {
    bool operator() (int i,int j) { return (i<j);}
} ordering_now;

stats::stats(string file_report, map<int,int> registro_interno, int offset, string title)
{
    this->__name_report = file_report;
    this->__title = title;
    this->__registro_interno = registro_interno;
    this->__offset = offset;
    this->TrueNegative = 0;
    this->TruePositive = 0;
    this->FalseNegative = 0;
    this->FalsePositive = 0;
    this->calc();
}

void stats::calc(){
    this->searchReport(this->__name_report); //fill this->report
    
    vector<int> immagini_n,immagini_all;
    for(map<int,int>::iterator it = this->__registro_interno.begin(); it != this->__registro_interno.end(); it++){
        int a = boost::lexical_cast<int>(it->first); //immagine
        immagini_all.push_back(a);
        immagini_n.push_back(a);
    }
   
    //valuto positivi
    for(map<int,int>::iterator it = this->__report.begin(); it!=this->__report.end(); it++){
        std::map<int,int>::iterator it_1,it_2;
        it_1 = this->__registro_interno.find(it->first); //loop
        it_2 = this->__registro_interno.find(it->second);        
        
        int z1 = it_1->second;
        int z2 = it_2->second;        
        if (z1 == z2){
            this->TruePositive++;   
            //cout << "TP: "<< it->first << " = " << it->second << endl;
            //creo negativi togliendo tutti i positivi che incrementalmente sto valutando                   
        }else{
            this->FalsePositive++;
        }
        immagini_n.erase(std::remove(immagini_n.begin(), immagini_n.end(), it->first), immagini_n.end());         
    }
    for(int i=0;i<immagini_n.size();i++){
        bool found = false; 
        if(immagini_n[i] < this->__offset){this->TrueNegative++; continue;}
        int yyy = immagini_n[i] - this->__offset;
        if (yyy < 0) {yyy = 0;}
        for(int j = yyy; j >= 0 ; j--){
            //cout << immagini_n[i] << " - " << immagini_all[j] << " = " <<immagini_n[i]-immagini_all[j]<< " (" << j << ") "<<endl;
            if(immagini_n[i]-immagini_all[j] >= this->__offset || yyy == 1){
                std::map<int,int>::iterator it_1,it_2;                
                it_1 = this->__registro_interno.find(immagini_n[i]); //loop
                it_2 = this->__registro_interno.find(immagini_all[j]);                
                int z1 = it_1->second;
                int z2 = it_2->second;   
                if(z1==z2){
                    found=true;
                    break;
                }
            }
        }  
        if (!found){
            this->TrueNegative++;
        }else{
            //cout << "FN" << immagini_n[i] << endl;            
            this->FalseNegative++;
        } 
    }    
}

void stats::searchReport(string pos)
{
    string line;
    ifstream Myfile;
    Myfile.open(pos.c_str());
    if(Myfile.is_open()) {
        while(Myfile.peek() != EOF) {
            vector<string> w1;
            getline(Myfile, line);
            boost::algorithm::trim(line);
            if(line.size() > 0) {
                //luogo               
                boost::algorithm::split(w1,line,boost::algorithm::is_any_of(":"));
                w1.erase( std::remove_if(w1.begin(), w1.end(), bind( &std::string::empty, _1 ) ), w1.end());                
                string w = w1[1];
                boost::algorithm::trim(w);
                string h = w1[0];
                int a = boost::lexical_cast<int>(h);
                int b = boost::lexical_cast<int>(w);
                (this->__report).insert(PairInt(a,b));
            }
        }
        Myfile.close();
    } else {
        cout<< pos <<" NON TROVATO."<<endl;
    }
}

string stats::toString(){    
    stringstream ss;
    ss << endl << this->__title << endl;
    ss << " ======== Statistiche (%) ========= "<< endl;
    ss << "Precision: " << this->getPrecision()*100 << endl;
    ss << "Recall: " << this->getRecall()*100 << endl;
    ss << "Accuracy: " << this->getAccuracy()*100 << endl;
    
    ss<<endl<<"TP:"<<this->TruePositive<<endl;
    ss<<"FP:"<<this->FalsePositive<<endl;
    ss<<"TN:"<<this->TrueNegative<<endl;
    ss<<"FN:"<<this->FalseNegative<<endl;
    return ss.str();
}

double stats::getAccuracy()
{
    return ((this->TruePositive+this->TrueNegative)/(this->TruePositive + this->TrueNegative +this->FalsePositive + this->FalseNegative));
}

double stats::getPrecision()
{
    return (this->TruePositive/(this->TruePositive+this->FalsePositive));
}

double stats::getRecall()
{
    return (this->TruePositive/(this->TruePositive+this->FalseNegative));
}

int stats::getTruePositive(){return this->TruePositive;}
int stats::getTrueNegative(){return this->TrueNegative;}
int stats::getFalsePositive(){return this->FalsePositive;}
int stats::getFalseNegative(){return this->FalseNegative;}

