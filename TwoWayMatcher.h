/*
 * File:   TwoWayMatcher.h
 * Author: thomas
 *
 * Created on 8 giugno 2011, 9.22
 */

#ifndef TWOWAYMATCHER_H
#define	TWOWAYMATCHER_H

#include <opencv2/opencv.hpp>



enum {
    TWM_BRUTEFORCE_L1, TWM_BRUTEFORCE_L1_INT, TWM_BRUTEFORCE_L2, TWM_FLANN
};

class TwoWayMatcher {
public:
    TwoWayMatcher(int type);
    TwoWayMatcher(const cv::Ptr<cv::DescriptorMatcher>& matcher1, const cv::Ptr<cv::DescriptorMatcher>& matcher2);
    TwoWayMatcher(const TwoWayMatcher& orig);
    virtual ~TwoWayMatcher();


    void match(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& matches,
            const cv::Mat& queryMask = cv::Mat(), const cv::Mat& trainMask = cv::Mat());
    void robustMatch(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& matches, 
            const cv::Mat& queryMask, const cv::Mat& trainMask);
    void robustMatchWNoRatioT(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, std::vector<cv::DMatch>& matches, 
            const cv::Mat& queryMask, const cv::Mat& trainMask);

private:

    float _ratio; //max ratio between 1st and 2nd NN
    bool refineF; //if true will refine F matrix
    double distance; // min distance to epipolar line
    double confidence; // confidence level (probability)
    int matcherType;
    cv::Ptr<cv::DescriptorMatcher> matcher1;
    cv::Ptr<cv::DescriptorMatcher> matcher2;

    inline virtual cv::Ptr<cv::DescriptorMatcher> clone(bool emptyTrainData = false) const {
    };

    int ratioTest(std::vector< std::vector<cv::DMatch> >& matches);
    int symmetryTest(
        const std::vector< std::vector<cv::DMatch> >& matches1, 
        const std::vector< std::vector<cv::DMatch> >& matches2, 
        std::vector<cv::DMatch>& symMatches);


};

#endif	/* TWOWAYMATCHER_H */
