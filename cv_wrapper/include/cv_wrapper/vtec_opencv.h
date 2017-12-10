#ifndef VTEC_COMPARISON_TESTS_VTEC_OPENCV_HEADER
#define VTEC_COMPARISON_TESTS_VTEC_OPENCV_HEADER

#include <vtec_core/image.h>
#include <vtec_core/types.h>
#include <homography_optimizer/types.h>
#include <opencv2/opencv.hpp>

namespace VTEC{

   /**
    * @brief      Converts a opencv image to vtec format
    *
    * @param[in]  image     The image
    * @param      vtec_img  The vtec image
    */
   void cv2vtecImg(const cv::Mat& image, VTEC::Image& vtec_img){
      cv::Mat img_copy;
      image.convertTo(img_copy,CV_8U); // ALWAYS DEEP COPY TO KEEP REF IMAGE 
      vtec_img.resize(image.cols, image.rows);
      size_t img_size = vtec_img.size();
      for(size_t i = 0; i < img_size; ++i){
         vtec_img.I->data[i] = img_copy.data[i];
      }
   }

   /**
    * @brief      Converts vtec image to opencv format
    *
    * @param[in]  vtec_img  The vtec image
    * @param      image     The image
    */
   void vtec2cvImg(const VTEC::Image& vtec_img, cv::Mat& image){
      image.create(vtec_img.I->rows, vtec_img.I->cols, CV_32F);
      size_t img_size = vtec_img.I->rows*vtec_img.I->cols;

      float * data = (float*) image.data;
      for(size_t i = 0; i < img_size; ++i){
         data[i] = (float) vtec_img.I->data[i]/255.0;
      }
   }

   /**
    * @brief      Converts vtec homography to opencv matrix
    *
    * @param[in]  vtec_h  The vtec h
    * @param      cv_h    The cv h
    */
   void vtec2cvHomography(const VTEC::Homography& vtec_h, cv::Mat& cv_h){
      cv_h.create(3,3,CV_64F);
      for(int i = 0; i < 3; i++){
         for(int j=0; j < 3; j++){
            cv_h.at<double>(j,i) = vtec_h[i+3*j];
         }
      }
   }

   /**
    * @brief      Converts opencv matrix to vtec homography 
    *
    * @param[in]  cv_h    The cv h
    * @param      vtec_h  The vtec h
    */
   void cv2vtecHomography(const cv::Mat& cv_h, VTEC::Homography& vtec_h){
      for(int i = 0; i < 3; i++){
         for(int j=0; j < 3; j++){
            vtec_h[i+3*j] = cv_h.at<double>(j,i);
         }
      }
   }

   /**
    * @brief      Warp a point with an homography matrix
    *
    * @param      p     { parameter_description }
    * @param[in]  H     { parameter_description }
    */
   void warpPoints(cv::Point2f& p, cv::Mat H){
      double w = p.x*H.at<double>(2,0) + p.y*H.at<double>(2,1) + H.at<double>(2,2);
      double u2 = (p.x*H.at<double>(0,0)+p.y*H.at<double>(0,1)+H.at<double>(0,2))/w;
      double v2 = (p.x*H.at<double>(1,0)+p.y*H.at<double>(1,1)+H.at<double>(1,2))/w;
      p.x = u2;
      p.y = v2;
   }


} /* namespace VTEC */

#endif /* VTEC_COMPARISON_TESTS_VTEC_OPENCV_HEADER */