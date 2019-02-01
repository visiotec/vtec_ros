#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <vtec_msgs/TrackingResult.h>
#include <homography_optimizer/ibg.h>

enum tracking_states
{
  STARTING = 0,
  NOT_TRACKING = 1,
  TRACKING = 2
};

VTEC::IBGHomographyOptimizer* ibg_optimizer;

cv::Mat H;
float alpha, beta;
image_transport::Publisher *annotated_pub_ptr, *stabilized_pub_ptr, *reference_pub_ptr;
ros::Publisher* results_pub_ptr;

int state = STARTING;
int BBOX_SIZE_X, BBOX_SIZE_Y;
int BBOX_POS_X, BBOX_POS_Y;

ros::Time last_ref_pub_time;
cv::Mat out_ref_template;

bool start_command = false;
cv::Mat cur_img;

int image_index = 0;

/**
 * @brief      Fills a vtec_msgs/TrackingResult message
 *
 * @param      msg          The message
 * @param[in]  score        The (zncc) score
 * @param[in]  H            The homography matrix
 * @param[in]  alpha        The photometric gain
 * @param[in]  beta         The photometric bias
 * @param[in]  bbox_size_x  The bounding box size x
 * @param[in]  bbox_size_y  The bounding box size y
 */
void fillTrackingMsg(vtec_msgs::TrackingResult& msg, const double score, const cv::Mat& H, const float alpha,
                     const float beta, int bbox_size_x, int bbox_size_y)
{
  cv::Point2f p1(0, 0), p2(0, bbox_size_y), p3(bbox_size_x, 0), p4(bbox_size_x, bbox_size_y);
  VTEC::warpPoints(p1, H);
  VTEC::warpPoints(p2, H);
  VTEC::warpPoints(p3, H);
  VTEC::warpPoints(p4, H);

  geometry_msgs::Point p;
  p.z = 0.0;
  p.x = p1.x;
  p.y = p1.y;
  msg.corners[0] = p;

  p.x = p2.x;
  p.y = p2.y;
  msg.corners[1] = p;

  p.x = p3.x;
  p.y = p3.y;
  msg.corners[2] = p;

  p.x = p4.x;
  p.y = p4.y;
  msg.corners[3] = p;

  msg.alpha = alpha;
  msg.beta = beta;
  msg.score = score;

  msg.homography[0] = H.at<double>(0, 0);
  msg.homography[1] = H.at<double>(0, 1);
  msg.homography[2] = H.at<double>(0, 2);
  msg.homography[3] = H.at<double>(1, 0);
  msg.homography[4] = H.at<double>(1, 1);
  msg.homography[5] = H.at<double>(1, 2);
  msg.homography[6] = H.at<double>(2, 0);
  msg.homography[7] = H.at<double>(2, 1);
  msg.homography[8] = H.at<double>(2, 2);
}

/**
 * @brief      Starts a tracking.
 */
void start_tracking()
{
  H = cv::Mat::eye(3, 3, CV_64F);
  H.at<double>(0, 2) = BBOX_POS_X;
  H.at<double>(1, 2) = BBOX_POS_Y;
  ibg_optimizer->setHomography(H);
  alpha = 1.0;
  beta = 0.0;

  ibg_optimizer->setReferenceTemplate(cur_img, BBOX_POS_X, BBOX_POS_Y, BBOX_SIZE_X, BBOX_SIZE_Y);
  cv::Mat reference_template;
  ibg_optimizer->getReferenceTemplate(reference_template);

  reference_template.convertTo(out_ref_template, CV_8U);
}

/**
 * @brief      Callback to handle incoming images
 *
 * @param[in]  msg   The message
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cur_img = cv_bridge::toCvShare(msg, "mono8")->image;
    vtec_msgs::TrackingResult result_msg;
    result_msg.header = msg->header;

    cv::Mat H_test = H.clone();
    float alpha_test = alpha;
    float beta_test = beta;

    double zncc = 0.0;

    bool force_reference_image_pub = false;

    if (start_command)
    {
      ROS_INFO("Reference Image Selected");
      start_command = false;
      start_tracking();
      force_reference_image_pub = true;
      state = TRACKING;
    }

    if (state != STARTING)
    {
      zncc = ibg_optimizer->optimize(cur_img, H_test, alpha_test, beta_test, VTEC::ZNCC_PREDICTOR);
    }

    if (state == NOT_TRACKING && zncc > 0.5 || state == TRACKING && zncc > 0.0)
    {
      state = TRACKING;
      H = H_test;
      alpha = alpha_test;
      beta = beta_test;

      cv::Mat current_template;
      ibg_optimizer->getCurrentTemplate(current_template);

      cv::Mat out_cur_template;
      current_template.convertTo(out_cur_template, CV_8U);
      sensor_msgs::ImagePtr stabilized_msg =
          cv_bridge::CvImage(std_msgs::Header(), "mono8", out_cur_template).toImageMsg();
      stabilized_pub_ptr->publish(stabilized_msg);

      VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y, cv::Scalar(0.0, 255.0, 0.0));
      fillTrackingMsg(result_msg, zncc, H, alpha, beta, BBOX_SIZE_X, BBOX_SIZE_Y);
      results_pub_ptr->publish(result_msg);

      if (ros::Time::now() - last_ref_pub_time > ros::Duration(5.0) || force_reference_image_pub)
      {
        sensor_msgs::ImagePtr reference_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", out_ref_template).toImageMsg();
        reference_pub_ptr->publish(reference_msg);
        last_ref_pub_time = ros::Time::now();
      }
    }
    else if (state != STARTING)
    {
      state = NOT_TRACKING;
    }

    if (state != TRACKING)
    {
      VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y);
      cv::putText(cur_img, "press S to start tracking", cv::Point(30, 60), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
                  3);
    }

    sensor_msgs::ImagePtr annotated_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cur_img).toImageMsg();
    annotated_msg->header.frame_id = "camera";
    annotated_pub_ptr->publish(annotated_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

/**
 * @brief      Callback for the keyboard command
 *
 * @param[in]  cmd_msg  The command message
 */
void cmdCallback(const std_msgs::Char cmd_msg)
{
  switch (cmd_msg.data)
  {
    // S key
    case 115:
      ROS_INFO("(re)starting tracking");
      start_command = true;
      break;
    default:
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ibgho_tracker_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  // Tracker Parameters
  int MAX_NB_ITERATION_PER_LEVEL;
  int MAX_NB_PYR_LEVEL;
  double PIXEL_KEEP_RATE;
  std::string reference_image_path;
  std::string image_topic = "usb_cam/image_raw";
  std::string homography_type = "full";

  nhPrivate.param<int>("bbox_pos_x", BBOX_POS_X, 200);
  nhPrivate.param<int>("bbox_pos_y", BBOX_POS_Y, 150);
  nhPrivate.param<int>("bbox_size_x", BBOX_SIZE_X, 200);
  nhPrivate.param<int>("bbox_size_y", BBOX_SIZE_Y, 200);
  nhPrivate.param<int>("max_nb_iter_per_level", MAX_NB_ITERATION_PER_LEVEL, 5);
  nhPrivate.param<int>("max_nb_pyr_level", MAX_NB_PYR_LEVEL, 2);
  nhPrivate.param<double>("sampling_rate", PIXEL_KEEP_RATE, 1.0);
  nhPrivate.getParam("image_topic", image_topic);
  nhPrivate.getParam("homography_type", homography_type);

  // Register publisher
  image_transport::ImageTransport it(nh);
  image_transport::Publisher annotated_pub = it.advertise("annotated_image", 1);
  annotated_pub_ptr = &annotated_pub;

  image_transport::Publisher stabilized_pub = it.advertise("stabilized_image", 1);
  stabilized_pub_ptr = &stabilized_pub;

  image_transport::Publisher reference_pub = it.advertise("reference_image", 1);
  reference_pub_ptr = &reference_pub;

  ros::Publisher results_pub = nh.advertise<vtec_msgs::TrackingResult>("tracking", 1);
  results_pub_ptr = &results_pub;

  image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);

  ros::Subscriber cmd_sub = nh.subscribe("track_cmd", 1, cmdCallback);

  // Initialize the optimizer according to the homography type
  if (homography_type == "affine")
  {
    ibg_optimizer = new VTEC::IBGAffineHomographyOptimizer();
  }
  else if (homography_type == "stretch")
  {
    ibg_optimizer = new VTEC::IBGStretchHomographyOptimizer();
  }
  else
  {
    ibg_optimizer = new VTEC::IBGFullHomographyOptimizer();
  }

  ibg_optimizer->initialize(MAX_NB_ITERATION_PER_LEVEL, MAX_NB_PYR_LEVEL, PIXEL_KEEP_RATE);

  // Start optimizer
  H = cv::Mat::eye(3, 3, CV_64F);
  H.at<double>(0, 2) = BBOX_POS_X;
  H.at<double>(1, 2) = BBOX_POS_Y;
  ibg_optimizer->setHomography(H);
  alpha = 1.0;
  beta = 0.0;

  last_ref_pub_time = ros::Time::now();

  ros::spin();
}
