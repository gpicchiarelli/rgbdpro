/*
 * File:   TwoWayMatcher.cpp
 * Author: thomas
 *
 * Created on 8 giugno 2011, 9.22
 */

#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "TwoWayMatcher.h"

using namespace std;
using namespace cv;

//ratio 0.65f
//distance 3.0
TwoWayMatcher::TwoWayMatcher(int type)
: matcherType(type), _ratio(0.45f), refineF(false),
confidence(0.99), distance(6.0) {
    switch (type) {
        case TWM_BRUTEFORCE_L2:
        {
            matcher1 = new cv::BFMatcher(NORM_L2); //cv::BruteForceMatcher< L2<float> >();
        }
            break;
        case TWM_BRUTEFORCE_L1_INT:
        {
            matcher1 = new cv::BFMatcher(NORM_L1); //cv::BruteForceMatcher< L1<int> >();
        }
            break;
        case TWM_FLANN:
        {
            matcher1 = new cv::FlannBasedMatcher();
        }
            break;
        default:
        {
            matcher1 = new cv::BFMatcher(NORM_L1); //cv::BruteForceMatcher< L1<float> >();
        }
    }

    matcher2 = matcher1->clone(true);
}

TwoWayMatcher::TwoWayMatcher(const Ptr<DescriptorMatcher>& firstMatcher, const Ptr<DescriptorMatcher>& secondMatcher) 
: _ratio(0.65f), refineF(true), confidence(0.99), distance(3.0), matcher1(firstMatcher), matcher2(secondMatcher){
    matcher1 = firstMatcher;
    matcher2 = secondMatcher;
}

TwoWayMatcher::TwoWayMatcher(const TwoWayMatcher& orig) {
}

TwoWayMatcher::~TwoWayMatcher() {
}

int TwoWayMatcher::ratioTest(vector< vector<DMatch> >& matches) {
    int removed = 0;
//    cout << "Intial match vector size: " << matches.size() << endl;
    // for all matches
    for (vector< vector<DMatch> >::iterator matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator) {
        // if 2 NN has been identified
        if (matchIterator->size() > 1) {
            // check distance ratio
            float currentRatio = (*matchIterator)[0].distance / (*matchIterator)[1].distance;
            if (currentRatio > _ratio) {
                matchIterator->clear(); // remove match
                removed++;
            }
        } else { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }


    }
//    cout << "Removed: " << removed << endl;
    return removed;
}

int TwoWayMatcher::symmetryTest(
        const vector< vector<DMatch> >& matches1,
        const vector< vector<DMatch> >& matches2,
        vector<DMatch>& symMatches) {
    // for all matches image 1 -> image 2
    for (vector< vector<DMatch> >::const_iterator matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1) {
        // ignore deleted matches
        if (matchIterator1->size() < 2)
            continue;
        // for all matches image 2 -> image 1
        for (vector< vector<DMatch> >::const_iterator matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2) {
            // ignore deleted matches
            if (matchIterator2->size() < 2)
                continue;
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
                    (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {
                // add symmetrical match
                symMatches.push_back(
                        cv::DMatch((*matchIterator1)[0].queryIdx,
                        (*matchIterator1)[0].trainIdx,
                        (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}

void TwoWayMatcher::robustMatchWNoRatioT(
        const Mat& queryDescriptors,
        const Mat& trainDescriptors,
        vector<DMatch>& matches,
        const Mat& queryMask,
        const Mat& trainMask) {

    // Match query descriptors with tranining and vice versa
    vector< vector<DMatch> > matches1;
    vector< vector<DMatch> > matches2;
    Ptr<DescriptorMatcher> tempMatcher1 = matcher1->clone(true);
    Ptr<DescriptorMatcher> tempMatcher2 = matcher2->clone(true);
//    tempMatcher1->add(vector<Mat > (1, trainDescriptors));
//    tempMatcher2->add(vector<Mat > (1, queryDescriptors));
//    tempMatcher1->knnMatch(queryDescriptors, matches1, 2);
//    tempMatcher2->knnMatch(trainDescriptors, matches2, 2);
//    cout << "qrows: " << queryDescriptors.size().height << " qcols: " << queryDescriptors.size().width << endl;
//    cout << "trows: " << trainDescriptors.size().height << " tcols: " << trainDescriptors.size().width << endl;
//    cout << "Q mask size: " << queryMask.rows << ", " << queryMask.cols << endl;
//    cout << "T mask size: " << trainMask.rows << ", " << trainMask.cols << endl;
    tempMatcher1->knnMatch(queryDescriptors, trainDescriptors, matches1, 2, queryMask);
    tempMatcher2->knnMatch(trainDescriptors, queryDescriptors, matches2, 2, trainMask);
    
   
    // Remove non-symmetrical matches
    symmetryTest(matches1, matches2, matches);
//    cout << "symmetry filtered matches size: " << matches.size() << endl;
}

void TwoWayMatcher::robustMatch(
        const Mat& queryDescriptors,
        const Mat& trainDescriptors,
        vector<DMatch>& matches,
        const Mat& queryMask,
        const Mat& trainMask) {

    // Match query descriptors with tranining and vice versa
    vector< vector<DMatch> > matches1;
    vector< vector<DMatch> > matches2;
    Ptr<DescriptorMatcher> tempMatcher1 = matcher1->clone(true);
    Ptr<DescriptorMatcher> tempMatcher2 = matcher2->clone(true);
    tempMatcher1->add(vector<Mat > (1, trainDescriptors));
    tempMatcher2->add(vector<Mat > (1, queryDescriptors));
    tempMatcher1->knnMatch(queryDescriptors, matches1, 2, vector<Mat > (1, queryMask));
    tempMatcher2->knnMatch(trainDescriptors, matches2, 2, vector<Mat > (1, trainMask));
    
    // Remove matches for which NN ratio is > than threshold
    int removed1 = ratioTest(matches1);
    int removed2 = ratioTest(matches2);
    
    // Remove non-symmetrical matches
    symmetryTest(matches1, matches2, matches);
//    cout << "symmetry filtered matches size: " << matches.size() << endl;
}

void TwoWayMatcher::match(const Mat& queryDescriptors, const Mat& trainDescriptors, vector<DMatch>& matches, const Mat& queryMask, const Mat& trainMask) {

    // Match query descriptors with tranining and vice versa
    vector<DMatch> matches1;
    vector<DMatch> matches2;
    Ptr<DescriptorMatcher> tempMatcher1 = matcher1->clone(true);
    Ptr<DescriptorMatcher> tempMatcher2 = matcher2->clone(true);
    tempMatcher1->add(vector<Mat > (1, trainDescriptors));
    tempMatcher2->add(vector<Mat > (1, queryDescriptors));
    tempMatcher1->match(queryDescriptors, matches1, vector<Mat > (1, queryMask));
    tempMatcher2->match(trainDescriptors, matches2, vector<Mat > (1, trainMask));

    // Find common matches
    int firstId, secondId;
    bool searchMirrorMatch;
    vector<DMatch>::iterator m1It;
    vector<DMatch>::iterator m2It;
    for (m1It = matches1.begin(); m1It != matches1.end(); m1It++) {

        firstId = (*m1It).queryIdx;
        secondId = (*m1It).trainIdx;

        for (m2It = matches2.begin(), searchMirrorMatch = true; (m2It != matches2.end()) && searchMirrorMatch; m2It++) {

            if ((*m2It).queryIdx == secondId) {
                if ((*m2It).trainIdx == firstId) {
                    // Add match to the matches set
                    matches.push_back(DMatch(firstId, secondId, (*m1It).distance));
                    searchMirrorMatch = false;
                } else {
                    searchMirrorMatch = false;
                }
            }

        }

    }

}
