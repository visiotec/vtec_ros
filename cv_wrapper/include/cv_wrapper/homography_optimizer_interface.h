#ifndef VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER
#define VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER

#include <cv_bridge/cv_bridge.h>

namespace VTEC
{
/**
 * @brief      Class for FULL homography optimizer interface. 8 degrees of freedom in the homography.
 */
class HomographyOptimizerInterface
{
public:
  /**
   * @brief      initializes the optimizer, building the image pyramid.
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
   * @param[in]  posx       The upper left x coordinate of the template
   * @param[in]  posy       The upper right x coordinate of the template
   * @param[in]  sizex      The length in x of the template
   * @param[in]  sizey      The length in y of the template
   *
   * @return     true if success, false otherwise.
   */
  virtual bool setReferenceTemplate(const cv::Mat& ref_image, const int posx, const int posy, const int sizex,
                                    const int sizey) = 0;

  /**
   * @brief      Optimization function
   *
   * @param[in]  curr_image  The current image
   * @param      H           The homography matrix.
   * @param      alpha       The alpha photometric parameter
   * @param      beta        The beta photometric parameter
   * @param[in]  predictor   The predictor
   *
   * @return     ZNCC score associated with the estimated parameters.
   */
  virtual double optimize(const cv::Mat& curr_image, cv::Mat& H, float& alpha, float& beta, int predictor) = 0;

  /**
   * @brief      Gets the number of iterations.
   *
   * @return     The number of iterations.
   */
  virtual std::vector<int> getNbIterations() = 0;

  /**
   * @brief      Gets the homography sequence.
   *
   * @return     The homography sequence.
   */
  virtual std::vector<cv::Mat> getHomographySequence() = 0;

}; /* class HomographyOptimizerInterface */

}  // namespace VTEC

#endif /* VTEC_CV_OPTIMIZER_HOMOGRAPHY_OPTIMIZER_INTERFACE_HEADER */