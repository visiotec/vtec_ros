#ifndef VTEC_CV_WRAPPER_IBG_CV_HEADER
#define VTEC_CV_WRAPPER_IBG_CV_HEADER

#include <cv_wrapper/homography_optimizer_interface.h>
#include <homography_optimizer/ibg_interface.h>
#include <homography_optimizer/types.h>
#include <cv_bridge/cv_bridge.h>

namespace VTEC
{
class IBGHomographyOptimizerCvWrapper : public HomographyOptimizerInterface
{
public:
  /**
   * @brief      Constructor
   */
  IBGHomographyOptimizerCvWrapper();

  /**
   * @brief      Initiliazation of the optimizer
   *
   * @param[in]  max_nb_iter       The maximum number of iterator
   * @param[in]  max_nb_pyr_level  The maximum number of pyr level
   * @param[in]  sampling_rate     The sampling rate
   */
  virtual void initialize(const int max_nb_iter, const int max_nb_pyr_level, const double sampling_rate) = 0;

  /**
   * @brief      Sets the reference template.
   *
   * @param[in]  ref_image  The reference image
   * @param[in]  posx       The upper left x coordinate of the template in the image
   * @param[in]  posy       The upper left y coordinate of the template in the image
   * @param[in]  sizex      The length in x of the template in the image.
   * @param[in]  sizey      The length in y of the template in the image.
   *
   * @return     true if success, false otherwise
   */
  bool setReferenceTemplate(const cv::Mat& ref_image, const int posx, const int posy, const int sizex, const int sizey);

  /**
   * @brief      Optimization function
   *
   * @param[in]  curr_image  The curr image
   * @param      H           The homography matrix
   * @param      alpha       The alpha
   * @param      beta        The beta
   * @param[in]  predictor   The predictor
   *
   * @return     the ZNCC score
   */
  double optimize(const cv::Mat& curr_image, cv::Mat& H, float& alpha, float& beta, int predictor);

  /**
   * @brief      Gets the number of iterations.
   *
   * @return     The number of iterations.
   */
  std::vector<int> getNbIterations();

  /**
   * @brief      Gets the homography sequence.
   *
   * @return     The homography sequence.
   */
  std::vector<cv::Mat> getHomographySequence();

  /**
   * @brief      Gets the current template.
   *
   * @param      img   The image
   */
  void getCurrentTemplate(cv::Mat& img);

  /**
   * @brief      Gets the reference template.
   *
   * @param      img   The image
   */
  void getReferenceTemplate(cv::Mat& img);

  /**
   * @brief      Sets the homography.
   *
   * @param[in]  H     the Homography matrix.
   */
  void setHomography(const cv::Mat H);

protected:
  IBGHomographyOptimizer* optimizer;

  bool initialized_;

}; /* class IBGHomographyOptimizerCvWrapper */

class IBGFullHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper
{
public:
  IBGFullHomographyOptimizerCvWrapper()
  {
  }

  void initialize(const int max_nb_iter, const int max_nb_pyr_level, const double sampling_rate)
  {
    optimizer = new IBGFullHomographyOptimizer();
    optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);
    initialized_ = true;
  }

}; /* class IBGFullHomographyOptimizerCvWrapper */

class IBGAffineHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper
{
public:
  IBGAffineHomographyOptimizerCvWrapper()
  {
  }

  void initialize(const int max_nb_iter, const int max_nb_pyr_level, const double sampling_rate)
  {
    optimizer = new IBGAffineHomographyOptimizer();
    optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);
    initialized_ = true;
  }

}; /* class IBGAffineHomographyOptimizerCvWrapper */

class IBGStretchHomographyOptimizerCvWrapper : public IBGHomographyOptimizerCvWrapper
{
public:
  IBGStretchHomographyOptimizerCvWrapper()
  {
  }

  void initialize(const int max_nb_iter, const int max_nb_pyr_level, const double sampling_rate)
  {
    optimizer = new IBGStretchHomographyOptimizer();
    optimizer->initialize(max_nb_iter, max_nb_pyr_level, sampling_rate);
    initialized_ = true;
  }

}; /* class IBGStretchHomographyOptimizerCvWrapper */
} /* namespace VTEC */

#endif /* VTEC_CV_WRAPPER_IBG_CV_HEADER */