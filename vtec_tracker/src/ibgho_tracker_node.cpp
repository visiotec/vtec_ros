#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_wrapper/ibg_cv.h>
#include <cv_wrapper/vtec_opencv.h>
#include <cv_wrapper/draw.h>
#include <vtec_msgs/TrackingResult.h>

enum tracking_states{
   NOT_TRACKING,
   TRACKING
};

VTEC::IBGFullHomographyOptimizerCvWrapper ibg_optimizer;
cv::Mat H;
float alpha, beta;
image_transport::Publisher *annotated_pub_ptr, *stabilized_pub_ptr, *reference_pub_ptr;
ros::Publisher* results_pub_ptr;

int state = NOT_TRACKING;
int BBOX_SIZE_X, BBOX_SIZE_Y;

ros::Time last_ref_pub_time;
cv::Mat out_ref_template;


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
void fillTrackingMsg(vtec_msgs::TrackingResult& msg, const double score, 
   const cv::Mat& H, const float alpha, const float beta, 
   int bbox_size_x, int bbox_size_y)
{

   cv::Point p1(0,0), p2(0, bbox_size_y), p3(bbox_size_x, 0), p4(bbox_size_x,bbox_size_y);
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

   msg.homography[0] = H.at<double>(0,0);
   msg.homography[1] = H.at<double>(0,1);
   msg.homography[2] = H.at<double>(0,2);
   msg.homography[3] = H.at<double>(1,0);
   msg.homography[4] = H.at<double>(1,1);
   msg.homography[5] = H.at<double>(1,2);
   msg.homography[6] = H.at<double>(2,0);
   msg.homography[7] = H.at<double>(2,1);
   msg.homography[8] = H.at<double>(2,2);

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
      cv::Mat cur_img = cv_bridge::toCvShare(msg, "mono8")->image;
      vtec_msgs::TrackingResult result_msg;
      result_msg.header = msg->header;

      cv::Mat H_test = H.clone();
      float alpha_test = alpha;
      float beta_test = beta;

      double zncc = 0.0;
      
      zncc = ibg_optimizer.optimize(cur_img, H_test, alpha_test, beta_test, VTEC::ZNCC_PREDICTOR);

      if(state== NOT_TRACKING && zncc>0.7 || state == TRACKING && zncc > 0.4){

         state = TRACKING;
         H = H_test;
         alpha = alpha_test;
         beta = beta_test;

         cv::Mat current_template;
         ibg_optimizer.getCurrentTemplate(current_template);

         cv::Mat out_cur_template;
         current_template.convertTo(out_cur_template, CV_8U, 255.0);
         sensor_msgs::ImagePtr stabilized_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", out_cur_template).toImageMsg();
         stabilized_pub_ptr->publish(stabilized_msg);

         VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y, cv::Scalar(0.0, 255.0, 0.0) );
         fillTrackingMsg(result_msg, zncc, H, alpha, beta, BBOX_SIZE_X, BBOX_SIZE_Y);
         results_pub_ptr->publish(result_msg);
         
      }else{
         state = NOT_TRACKING;
         VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y);
      }

      sensor_msgs::ImagePtr annotaded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cur_img).toImageMsg();
      annotaded_msg->header.frame_id = "camera";
      annotated_pub_ptr->publish(annotaded_msg);

      if(ros::Time::now() - last_ref_pub_time > ros::Duration(5.0)){
         sensor_msgs::ImagePtr reference_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", out_ref_template).toImageMsg();
         reference_pub_ptr->publish(reference_msg);
         last_ref_pub_time = ros::Time::now();
      }
      
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ibgho_tracker_node");
   ros::NodeHandle nh;
   ros::NodeHandle nhPrivate("~");

   // Tracker Parameters
   int BBOX_POS_X, BBOX_POS_Y;
   int MAX_NB_ITERATION_PER_LEVEL;
   int MAX_NB_PYR_LEVEL;
   double PIXEL_KEEP_RATE;
   std::string reference_image_path;
   std::string image_topic = "camera/image";

   nhPrivate.param<int>("bbox_pos_x", BBOX_POS_X, 200);
   nhPrivate.param<int>("bbox_pos_y", BBOX_POS_Y, 150);
   nhPrivate.param<int>("bbox_size_x", BBOX_SIZE_X, 200);
   nhPrivate.param<int>("bbox_size_y", BBOX_SIZE_Y, 200);
   nhPrivate.param<int>("max_nb_iter_per_level", MAX_NB_ITERATION_PER_LEVEL, 5);
   nhPrivate.param<int>("max_nb_pyr_level", MAX_NB_PYR_LEVEL, 2);
   nhPrivate.param<double>("sampling_rate", PIXEL_KEEP_RATE, 1.0);
   nhPrivate.getParam("reference_image_path", reference_image_path);
   nhPrivate.getParam("image_topic", image_topic);

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

   // Start optimizer 
   ibg_optimizer.initialize(MAX_NB_ITERATION_PER_LEVEL, MAX_NB_PYR_LEVEL, PIXEL_KEEP_RATE);
   
   // Set reference template
   ROS_INFO_STREAM("Reference Path: " << reference_image_path);
   cv::Mat base_img = cv::imread(reference_image_path, CV_LOAD_IMAGE_GRAYSCALE);
   ibg_optimizer.setReferenceTemplate(base_img, BBOX_POS_X, BBOX_POS_Y, BBOX_SIZE_X, BBOX_SIZE_Y);

   // Display the reference template
   cv::Mat reference_template;
   ibg_optimizer.getReferenceTemplate(reference_template);

   reference_template.convertTo(out_ref_template, CV_8U, 255.0);

   // Initialize optimization variables
   H = cv::Mat::eye(3,3,CV_64F);
   H.at<double>(0,2) = 200;
   H.at<double>(1,2) = 200;
   ibg_optimizer.setHomography(H);
   alpha = 1.0;
   beta = 0.0;
   last_ref_pub_time = ros::Time::now();

   ros::spin();
}


