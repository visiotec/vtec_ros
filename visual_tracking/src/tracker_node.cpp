#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_optimizer/ibg_full.h>
#include <cv_optimizer/draw.h>
#include <visual_tracking/TrackingResult.h>

enum tracking_states{
   NOT_TRACKING,
   TRACKING
};

VTEC::CvIBGFullHomographyOptimizer cv_ibg_optimizer;
cv::Mat H;
float alpha, beta;
image_transport::Publisher *annotated_pub_ptr, *stabilized_pub_ptr;
ros::Publisher* results_pub_ptr;

int skipFactor=5;
int skipNumber = 0;

int state = NOT_TRACKING;
int BBOX_SIZE_X, BBOX_SIZE_Y;


void fillTrackingMsg(visual_tracking::TrackingResult& msg, const double score, 
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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   

   try
   {
      cv::Mat cur_img = cv_bridge::toCvShare(msg, "mono8")->image;
      visual_tracking::TrackingResult result_msg;
      result_msg.header = msg->header;

      cv::Mat H_test = H.clone();
      float alpha_test = alpha;
      float beta_test = beta;

      double zncc;
      
      zncc = cv_ibg_optimizer.optimize(cur_img, H_test, alpha_test, beta_test, ZNCC_PREDICTOR);

      if(zncc>0.7){

         state = TRACKING;
         H = H_test;
         alpha = alpha_test;
         beta = beta_test;

         cv::Mat current_template;
         cv_ibg_optimizer.getCurrentTemplate(current_template);

         cv::Mat out_cur_template;
         current_template.convertTo(out_cur_template, CV_8U);
         sensor_msgs::ImagePtr stabilized_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", out_cur_template).toImageMsg();
         stabilized_pub_ptr->publish(stabilized_msg);

         VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y, cv::Scalar(0.0, 255.0, 0.0) );

      }else{
         state = NOT_TRACKING;
         VTEC::drawResult(cur_img, H, zncc, BBOX_SIZE_X, BBOX_SIZE_Y);
      }

      ROS_INFO_STREAM("ZNCC Score: " << zncc);
      ROS_INFO_STREAM("H: " << H);
      ROS_INFO_STREAM("alpha: " << alpha << ", beta: " << beta);
      fillTrackingMsg(result_msg, zncc, H, alpha, beta, BBOX_SIZE_X, BBOX_SIZE_Y);

      results_pub_ptr->publish(result_msg);

      sensor_msgs::ImagePtr annotaded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cur_img).toImageMsg();
      annotated_pub_ptr->publish(annotaded_msg);

   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "image_listener");
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

   // // Register publisher
   image_transport::ImageTransport it(nh);
   image_transport::Publisher annotated_pub = it.advertise("annotated_image", 1);
   annotated_pub_ptr = &annotated_pub;

   image_transport::Publisher stabilized_pub = it.advertise("stabilized_image", 1);
   stabilized_pub_ptr = &stabilized_pub;

   ros::Publisher results_pub = nh.advertise<visual_tracking::TrackingResult>("tracking", 1);
   results_pub_ptr = &results_pub;

   // Start optimizer 
   cv_ibg_optimizer.initialize(MAX_NB_ITERATION_PER_LEVEL, MAX_NB_PYR_LEVEL, PIXEL_KEEP_RATE);
   
   // Set reference template
   ROS_INFO_STREAM("Reference Path: " << reference_image_path);
   cv::Mat base_img = cv::imread(reference_image_path, CV_LOAD_IMAGE_GRAYSCALE);
   cv_ibg_optimizer.setReferenceTemplate(base_img, BBOX_POS_X, BBOX_POS_Y, BBOX_SIZE_X, BBOX_SIZE_Y);

   // //Display the reference template
   cv::Mat reference_template;
   cv_ibg_optimizer.getReferenceTemplate(reference_template);

   H = cv::Mat::eye(3,3,CV_64F);
   cv_ibg_optimizer.setHomography(H);
   // Initialize optimization variables
   // cv_ibg_optimizer.getHomography(H);
   alpha = 1.0;
   beta = 0.0;

   // Create window and spin
   cv::namedWindow("reference_template");
   cv::startWindowThread();
   cv::imshow("reference_template", reference_template/255.0);
   cv::waitKey(0);

   // image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
   ros::spin();
   cv::destroyWindow("view");
}


