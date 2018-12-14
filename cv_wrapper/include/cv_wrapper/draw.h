#ifndef VTEC_CV_WRAPPER_DRAW_H
#define VTEC_CV_WRAPPER_DRAW_H
namespace VTEC
{
/**
 * @brief      Warp a point using an homography matrix
 *
 * @param      p     The point
 * @param[in]  H     The homography matrix
 */
void warpPoints(cv::Point& p, cv::Mat H)
{
  double w = p.x * H.at<double>(2, 0) + p.y * H.at<double>(2, 1) + H.at<double>(2, 2);
  double u2 = (p.x * H.at<double>(0, 0) + p.y * H.at<double>(0, 1) + H.at<double>(0, 2)) / w;
  double v2 = (p.x * H.at<double>(1, 0) + p.y * H.at<double>(1, 1) + H.at<double>(1, 2)) / w;
  p.x = u2;
  p.y = v2;
}

/**
 * @brief      Draws a tracking result in an image.
 *
 * @param      image        The image
 * @param[in]  H            The homography matrix
 * @param[in]  score        The score
 * @param[in]  bbox_size_x  The bounding box size x
 * @param[in]  bbox_size_y  The bounding box size y
 * @param[in]  color        The color
 */
void drawResult(cv::Mat& image, cv::Mat H, double score, int bbox_size_x, int bbox_size_y,
                cv::Scalar color = cv::Scalar(255, 255, 255))
{
  cv::Point p1(0, 0), p2(0, bbox_size_y), p3(bbox_size_x, 0), p4(bbox_size_x, bbox_size_y);
  warpPoints(p1, H);
  warpPoints(p2, H);
  warpPoints(p3, H);
  warpPoints(p4, H);
  cv::line(image, p1, p2, color, 2);
  cv::line(image, p2, p4, color, 2);
  cv::line(image, p4, p3, color, 2);
  cv::line(image, p3, p1, color, 2);
  cv::putText(image, "score: " + std::to_string(score), cv::Point(30, 30), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
              3);
}

} /* namespace VTEC */
#endif /* VTEC_CV_WRAPPER_DRAW_H */
