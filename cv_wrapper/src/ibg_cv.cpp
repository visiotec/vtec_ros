#include <cv_wrapper/ibg_cv.h>
#include <vtec_core/image.h>
#include <homography_optimizer/types.h>
#include <cv_wrapper/vtec_opencv.h>

VTEC::IBGHomographyOptimizerCvWrapper::IBGHomographyOptimizerCvWrapper()
{
  initialized_ = false;
}

bool VTEC::IBGHomographyOptimizerCvWrapper::setReferenceTemplate(const cv::Mat& ref_image, const int posx,
                                                                 const int posy, const int sizex, const int sizey)
{
  if (!initialized_)
    return false;
  /* convert cv Image to vtec image */
  VTEC::Image v_image;
  VTEC::cv2vtecImg(ref_image, v_image);

  initialized_ = optimizer->setReferenceTemplate(v_image, posx, posy, sizex, sizey);
  return initialized_;
}

double VTEC::IBGHomographyOptimizerCvWrapper::optimize(const cv::Mat& curr_image, cv::Mat& H, float& alpha, float& beta,
                                                       int predictor)
{
  if (!initialized_)
    return -1.0;

  /* convert cv Image to Vtec image */
  VTEC::Image v_image;
  VTEC::cv2vtecImg(curr_image, v_image);

  /* convert cv Homography to Vtec Homography */
  VTEC::Homography v_H;
  v_H[0] = H.at<double>(0, 0);
  v_H[1] = H.at<double>(0, 1);
  v_H[2] = H.at<double>(0, 2);
  v_H[3] = H.at<double>(1, 0);
  v_H[4] = H.at<double>(1, 1);
  v_H[5] = H.at<double>(1, 2);
  v_H[6] = H.at<double>(2, 0);
  v_H[7] = H.at<double>(2, 1);
  v_H[8] = H.at<double>(2, 2);

  double score = optimizer->optimize(v_image, v_H, alpha, beta, predictor);

  H.at<double>(0, 0) = v_H[0];
  H.at<double>(0, 1) = v_H[1];
  H.at<double>(0, 2) = v_H[2];
  H.at<double>(1, 0) = v_H[3];
  H.at<double>(1, 1) = v_H[4];
  H.at<double>(1, 2) = v_H[5];
  H.at<double>(2, 0) = v_H[6];
  H.at<double>(2, 1) = v_H[7];
  H.at<double>(2, 2) = v_H[8];

  return score;
}

std::vector<int> VTEC::IBGHomographyOptimizerCvWrapper::getNbIterations()
{
  if (!initialized_)
  {
    std::vector<int> v;
    return v;
  }
  else
  {
    return optimizer->getNbIterations();
  }
};

std::vector<cv::Mat> VTEC::IBGHomographyOptimizerCvWrapper::getHomographySequence()
{
  auto vtec_homographies = optimizer->getHomographySequence();
  size_t size = vtec_homographies.size();
  std::vector<cv::Mat> cv_homographies;

  for (int i = 0; i < size; ++i)
  {
    cv::Mat cv_H;
    VTEC::Homography* vtec_H = vtec_homographies[i];

    VTEC::vtec2cvHomography(*vtec_H, cv_H);

    cv_homographies.push_back(cv_H);
  }
  return cv_homographies;
}

void VTEC::IBGHomographyOptimizerCvWrapper::setHomography(const cv::Mat H)
{
  VTEC::Homography vtec_h;
  cv2vtecHomography(H, vtec_h);

  return optimizer->setHomography(vtec_h);
}

void VTEC::IBGHomographyOptimizerCvWrapper::getCurrentTemplate(cv::Mat& img)
{
  VTEC::Image vtec_template;
  optimizer->getCurrentTemplate(vtec_template);
  vtec2cvImg(vtec_template, img);
}

void VTEC::IBGHomographyOptimizerCvWrapper::getReferenceTemplate(cv::Mat& img)
{
  VTEC::Image vtec_template;
  optimizer->getReferenceTemplate(vtec_template);
  vtec2cvImg(vtec_template, img);
}