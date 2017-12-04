#ifndef VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER
#define VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER

#include <cv_bridge/cv_bridge.h>

namespace VTEC{

    /**
      * @brief      Class for FULL homography optimizer interface. 8 degrees of freedom in the homography.
      */
    class HomographyOptimizerInterface{
    public:

      virtual void initialize(const int max_nb_iter,const int max_nb_pyr_level,const double sampling_rate)=0;

      virtual bool setReferenceTemplate(const cv::Mat& ref_image, 
        const int posx, const int posy, const int sizex, const int sizey)=0;
            
      virtual double optimize(const cv::Mat & curr_image, cv::Mat& H, float & alpha, float & beta, int predictor)=0;

      virtual std::vector<int> getNbIterations()=0;

      virtual std::vector<cv::Mat> getHomographySequence()=0;

    }; /* class HomographyOptimizerInterface */

} /* end namespace */

#endif /* VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER */