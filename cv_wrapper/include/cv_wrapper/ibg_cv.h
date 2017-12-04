#ifndef VTEC_CV_WRAPPER_IBG_CV_HEADER
#define VTEC_CV_WRAPPER_IBG_CV_HEADER

#include <cv_wrapper/homography_optimizer_interface.h>
#include <homography_optimizer/ibg_interface.h>
#include <homography_optimizer/types.h>
#include <cv_bridge/cv_bridge.h>

namespace VTEC{

   class IBGHomographyOptimizerCvWrapper : public HomographyOptimizerInterface{
   public:

      IBGHomographyOptimizerCvWrapper();

      virtual void initialize(const int max_nb_iter,const int max_nb_pyr_level,const double sampling_rate)=0; 

      bool setReferenceTemplate(const cv::Mat& ref_image, 
        const int posx, const int posy, const int sizex, const int sizey);
            
      double optimize(const cv::Mat & curr_image, cv::Mat& H, float & alpha, float & beta, int predictor);

      std::vector<int> getNbIterations();

      std::vector<cv::Mat> getHomographySequence();

      void getCurrentTemplate(cv::Mat& img);
  
      void getReferenceTemplate(cv::Mat& img);    

      void setHomography(const cv::Mat H);      

   protected:

      IBGHomographyOptimizer* optimizer;

      bool initialized_;

   }; /* class IBGHomographyOptimizerCvWrapper */

   class IBGFullHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper{
   public:

      IBGFullHomographyOptimizerCvWrapper(){}

      void initialize(const int max_nb_iter,const int max_nb_pyr_level,const double sampling_rate){
            optimizer = new IBGFullHomographyOptimizer();
            optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);
            initialized_ = true;
      }

   }; /* class IBGFullHomographyOptimizerCvWrapper */

   class IBGAffineHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper{
   public:

      IBGAffineHomographyOptimizerCvWrapper(){}

      void initialize(const int max_nb_iter,const int max_nb_pyr_level,const double sampling_rate){
            optimizer = new IBGAffineHomographyOptimizer();
            optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);            
            initialized_ = true;
      }

   }; /* class IBGAffineHomographyOptimizerCvWrapper */

   class IBGStretchHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper{
   public:

      IBGStretchHomographyOptimizerCvWrapper(){}

      void initialize(const int max_nb_iter,const int max_nb_pyr_level,const double sampling_rate){
            optimizer = new IBGStretchHomographyOptimizer();
            optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);
            initialized_ = true;
      }

   }; /* class IBGStretchHomographyOptimizerCvWrapper */   
   

} /* namespace VTEC */

#endif /* VTEC_CV_WRAPPER_IBG_CV_HEADER */