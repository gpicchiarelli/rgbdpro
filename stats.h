/**
 * File: stats.h
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

#ifndef STATS_H
#define STATS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <pthread.h>
#include <iostream> 
#include <sstream> 
#include <fstream>
#include <assert.h>

#include <boost/thread/thread.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp> 
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/export.hpp> 
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

class stats
{
public:
    typedef pair <int, int> PairInt;
    stats(string file_report, map<int, int> registro_interno, int offset, string title);
    int getTruePositive();
    int getTrueNegative();
    int getFalsePositive();
    int getFalseNegative();
    double getPrecision();
    double getRecall();
    double getAccuracy();
    string toString();
    void searchReport(string pos);
    
private:
    string __name_report,__title;
    map<int, int> __report;    
    map<int,int> __registro_interno;    
    void readReport();
    void calc();
    double TruePositive;
    double TrueNegative;
    double FalsePositive;
    double FalseNegative;
    int __offset;
};

#endif // STATS_H
