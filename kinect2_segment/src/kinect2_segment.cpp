/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <stack>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

//using namespace std;
//using namespace cv;

 std::mutex lock;
//
  std::atomic_int mouseX;
  std::atomic_int mouseY;
  std::atomic_int mouseBtnType;

  void onMouse(int event, int x, int y, int flags, void* ustc) 
  {
   // std::cout << "onMouse: " << x << " " << y << std::endl;
    mouseX  = x;
    mouseY  = y;
    mouseBtnType = event;
   }
//



class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
 
  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
//
  ros::Publisher leftBtnPointPub =
    nh.advertise<geometry_msgs::PointStamped>("/kinect2/click_point/left", 1);
  ros::Publisher rightBtnPointPub =
    nh.advertise<geometry_msgs::PointStamped>("/kinect2/click_point/right", 1);
  ros::Publisher pcl_pub =
    nh.advertise<geometry_msgs::PoseStamped>("/kinect2/segment/point_orientation", 1);

//

  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

        dispDepth(depth, depthDisp, 12000.0f);
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        cv::imshow("Image Viewer", combined);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

//离群点
pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(p_cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.5);//单位m
	sor.filter(*pcloud_filtered);

	std::cout << "PointCloud after StatisticalOutlierRemoval: " << pcloud_filtered->width * pcloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*pcloud_filtered) << ")." << endl << endl;

	//sor.setNegative(true);
	//sor.filter(*pcloud_filtered);
	return pcloud_filtered;
}


//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
void regionGrowing(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pcloud, int mouse_x, int mouse_y) const
  {
    //result
    cv::Mat depth_res = cv::Mat::zeros(depth.rows, depth.cols, CV_8U);
    cv::Mat color_res = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);
    // 用于标记是否遍历过某点 
    cv::Mat flagMat; 
    depth_res.copyTo(flagMat);  
    float threshold = 0.005;//阈值，单位：m

    //opencv Point类
    cv::Point connects[8] = {cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1), cv::Point(1, 0), cv::Point(1, 1), cv::Point(0, 1), cv::Point(-1, 1), cv::Point(-1, 0)};  
  
    //区域增长后点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_res(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcloud_res->height = depth.rows;
    pcloud_res->width = depth.cols;
    pcloud_res->is_dense = false;
    pcloud_res->points.resize(pcloud_res->height * pcloud_res->width);
        
    // 初始化种子点 
    std::stack<cv::Point> seeds; 
    seeds.push(cv::Point(mouse_x, mouse_y));  //种子点入栈
    depth_res.at<uchar>(mouse_y, mouse_x)= depth.at<uchar>(mouse_y, mouse_x);  //at(i,j)行序号,列序号
    color_res.at<cv::Vec3b>(mouse_y, mouse_x)[0] = color.at<cv::Vec3b>(mouse_y, mouse_x)[0];
    color_res.at<cv::Vec3b>(mouse_y, mouse_x)[1] = color.at<cv::Vec3b>(mouse_y, mouse_x)[1];
    color_res.at<cv::Vec3b>(mouse_y, mouse_x)[2] = color.at<cv::Vec3b>(mouse_y, mouse_x)[2];

    size_t m  = 0;//点云计数


    while (!seeds.empty()) 
    { 
        cv::Point seed = seeds.top(); 
        seeds.pop(); 
        //std::cout << seeds.top() << std::endl;


        // 标记为已遍历过的点 
        flagMat.at<int>(seed.y, seed.x) = 1; 
    
        // 遍历8邻域 
        for (size_t i = 0; i < 8; i++) 
        { 
            int tmpx = seed.x + connects[i].x; 
            int tmpy = seed.y + connects[i].y; 

            if (flagMat.at<uchar>(tmpy, tmpx) == 1 || tmpx < 0 || tmpy < 0 || tmpx >= depth.cols || tmpy >= depth.rows || depth.at<uchar>(tmpy, tmpx) == std::numeric_limits<float>::quiet_NaN() )  
            {
                flagMat.at<uchar>(tmpy, tmpx) = 1;
                //continue;  
            }
            // 没有被标记过的点 
            if ( flagMat.at<uchar>(tmpy, tmpx) == 0 ) 
            {   
                //深度在阈值内
                if ( ((pcloud->points[tmpy * depth.cols + tmpx].z - pcloud->points[seed.y * depth.cols + seed.x].z) > (-threshold)) && ((pcloud->points[tmpy * depth.cols + tmpx].z - pcloud->points[seed.y * depth.cols + seed.x].z) < threshold) )
                {    
                flagMat.at<uchar>(tmpy, tmpx) = 1; // 标记 
                seeds.push(cv::Point(tmpx, tmpy)); // 种子压栈 
         
                depth_res.at<uchar>(tmpy, tmpx) = depth.at<uchar>(tmpy, tmpx); // 深度生长 
                color_res.at<cv::Vec3b>(tmpy, tmpx)[0] = color.at<cv::Vec3b>(tmpy, tmpx)[0];//RGB生长
                color_res.at<cv::Vec3b>(tmpy, tmpx)[1] = color.at<cv::Vec3b>(tmpy, tmpx)[1];
                color_res.at<cv::Vec3b>(tmpy, tmpx)[2] = color.at<cv::Vec3b>(tmpy, tmpx)[2];
                          
                //构建点云
                //const pcl::PointXYZRGBA & pt = pcloud->points[tmpy * depth.cols + tmpx];
                //pcloud_res->points[m].x = tmpx;
                //pcloud_res->points[m].y = tmpy;
                //pcloud_res->points[m].z = pt.z;
                std::cout << m << ": " << std::endl;
                //std::cout << pcloud_res->points[m].x << std::endl;
                //std::cout << pcloud_res->points[m].y << std::endl;
                //std::cout << pcloud_res->points[m].z << std::endl;  
                m++;
                }
            } 
        }
    } 

    std::cout << pcloud->height * pcloud->width << endl;

    //可视化点云
    //pcl::visualization::CloudViewer viewer("Segment Point Cloud Viewer");
    //viewer.showCloud(pcloud_res);
    //while (!viewer.wasStopped())
    //{
    //}
    createCloud(depth, color_res, pcloud_res);
    std::cout << pcloud_res->height * pcloud_res->width << endl;

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addPointCloud(pcloud_res, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    while (!visualizer->wasStopped())
    {
       visualizer->spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    visualizer->close();

  } 
  
void regionGrowing1(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pcloud, int mouse_x, int mouse_y) const
{
		//result
		cv::Mat depth_res = cv::Mat::zeros(depth.rows, depth.cols, CV_8U);
                cv::Mat color_res = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);

	        float threshold = 0.03;//阈值，单位：m;2cm

		//区域增长后点云
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_res(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcloud_res->height = depth.rows;
		pcloud_res->width = depth.cols;
		pcloud_res->is_dense = false;
		pcloud_res->points.resize(pcloud_res->height * pcloud_res->width);
		//float p = depth.at<uchar>(mouse_y, mouse_x);
                //std::cout << p << std::endl;

		size_t i = 0;//点云计数
                int m,n;

		for (m = -50; m < 50; m++)
		{
			for (n = -50; n < 50; n++)
			{
                              if ( ((pcloud->points[(mouse_y+n)* depth.cols + (mouse_x+m)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) > (-threshold)) && ((pcloud->points[(mouse_y+n)* depth.cols + (mouse_x+m)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) < threshold) )

				//if ((depth.at<uchar>(mouse_y + n, mouse_x + m) - depth.at<uchar>(mouse_y, mouse_x)) > (-threshold) && (depth.at<uchar>(mouse_y + n, mouse_x + m) - depth.at<uchar>(mouse_y, mouse_x)) < threshold)
				{
				depth_res.at<uchar>(mouse_y + n, mouse_x + m) = depth.at<uchar>(mouse_y + n, mouse_x + m);
				color_res.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[0] = color.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[0];
				color_res.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[1] = color.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[1];
				color_res.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[2] = color.at<cv::Vec3b>(mouse_y + n, mouse_x + m)[2];
				i++;
                                std::cout << i << ": " << std::endl;
				}
			}
		}
				
    createCloud(depth, color_res, pcloud_res);

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addPointCloud(pcloud_res, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    while (!visualizer->wasStopped())
    {
       visualizer->spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    visualizer->close();
  } 

void regionGrowingNeck(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pcloud, int mouse_x, int mouse_y) const
{
		//result
		cv::Mat depth_res = cv::Mat::zeros(depth.rows, depth.cols, CV_16U);
                cv::Mat color_res = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);
                float threshold = 0.03;//阈值，单位：m

		//区域增长后点云
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_res(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_res_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcloud_res->height = depth.rows;
		pcloud_res->width = depth.cols;
		pcloud_res->is_dense = false;
		pcloud_res->points.resize(pcloud_res->height * pcloud_res->width);
		
                
		size_t i = 0;//点云计数
                int p,q,width_res,hight_res; 
                q = 0;            
        	for (p = -50; p < 50; p++)
		{
                  if ( ((pcloud->points[(mouse_y)* depth.cols + (mouse_x+p)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) > (-threshold)) && ((pcloud->points[(mouse_y)* depth.cols + (mouse_x+p)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) < threshold) )
                  {
		        q++;                       
		  }
		}
                width_res = q;
                hight_res = q*1.5;
                std::cout << width_res << std::endl;
                std::cout << hight_res << std::endl;

                int j = 0;
                for (int m = -hight_res*0.5; m < hight_res*0.5; m++)
		{
                        for (int n = -width_res*0.5; n < width_res*0.5; n++)
		 	{                                                        
                               if ( ((pcloud->points[(mouse_y + m)* depth.cols + (mouse_x + n)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) > (-threshold)) && ((pcloud->points[(mouse_y + m)* depth.cols + (mouse_x + n)].z - pcloud->points[mouse_y * depth.cols + mouse_x].z) < threshold) )
			        {
                                           depth_res.at<uint16_t>(mouse_y + m, mouse_x + n) = depth.at<uint16_t>(mouse_y + m, mouse_x + n);
                                           color_res.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[0] = color.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[0];
			                   color_res.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[1] = color.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[1];
                                           color_res.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[2] = color.at<cv::Vec3b>(mouse_y + m, mouse_x + n)[2];
                                           j++;
                                          
		             	}
			}
                       
                 }
    createCloud(depth_res, color, pcloud_res);
    
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "kinect2_link";

    //求法线
    //Eigen::Vector4f plane_parameters;
    //float curvature;
    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> >(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    //tree->setInputCloud(pcloud_res);
    normal_estimator.setSearchMethod(tree);                                                                
    normal_estimator.setRadiusSearch(0.01);
    normal_estimator.setInputCloud(pcloud_res);
    normal_estimator.setViewPoint(0, 0, 1.0);
    //normal_estimator.setKSearch(5);
    normal_estimator.compute(*normals);
    //normal_estimator.computePointNormal(pcloud_res,*normals,plane_parameters,curvature);
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //pcl::concatenateFields(*pcloud_res,*normals,*cloud_with_normals);*/    
   
              				                                              			
    cv::Mat depth_r = cv::Mat::zeros(depth.rows, depth.cols, CV_16U);
    cv::Mat depth_c = cv::Mat::zeros(depth.rows, depth.cols, CV_16U);
    cv::Mat color_r = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    int k = 0;
    for (int m = 0;m < depth_res.cols;m++)
    {
             if (depth_res.at<uint16_t>(mouse_y, m) != 0)
             {
             depth_r.at<uint16_t>(mouse_y , m) = depth_res.at<uint16_t>(mouse_y, m);
             color_r.at<cv::Vec3b>(mouse_y, m)[0] = color.at<cv::Vec3b>(mouse_y, m)[0];
	     color_r.at<cv::Vec3b>(mouse_y, m)[1] = color.at<cv::Vec3b>(mouse_y, m)[1];
             color_r.at<cv::Vec3b>(mouse_y, m)[2] = color.at<cv::Vec3b>(mouse_y, m)[2];
             k++;
             std::cout << k << ": " << std::endl;
            
             const pcl::PointXYZRGBA& pt_Msg = pcloud->points[mouse_y * depth_r.cols + m ];
             float normal_x = normals->points[mouse_y * depth_res.cols + m].data_n[0];
             float normal_y = normals->points[mouse_y * depth_res.cols + m].data_n[1];
             float normal_z = normals->points[mouse_y * depth_res.cols + m].data_n[2];
             float normal = sqrt(normal_x * normal_x  + normal_y * normal_y + normal_z * normal_z);
             float cos_a = normal_x/normal;
             float cos_b = normal_y/normal;
             float cos_c = normal_z/normal;
             float a = acos(cos_a);
             float b = acos(cos_b);
             float c = acos(cos_c);
             float s1 = sin(a/2);
             float s2 = sin(b/2);
             float s3 = sin(c/2);
             float c1 = cos(a/2);
             float c2 = cos(b/2);
             float c3 = cos(c/2);
             float orientation_x = s2*s3*c1 + c2*c3*s1;
             float orientation_y = s2*c3*c1 + c2*s3*s1;
             float orientation_z = c2*s3*c1 + s2*c3*s1;
             float orientation_w = c2*c3*c1 + s2*s3*s1;
             poseMsg.pose.position.x = pt_Msg.x;
             poseMsg.pose.position.y = pt_Msg.y;
             poseMsg.pose.position.z = pt_Msg.z;  
             poseMsg.pose.orientation.x = orientation_x;
             poseMsg.pose.orientation.y = orientation_y;
             poseMsg.pose.orientation.z = orientation_z;
             poseMsg.pose.orientation.w = orientation_w;
             /*std::cout << "normal_x: " << normal_x << std::endl;
             std::cout << "normal_y: " << normal_y << std::endl;
             std::cout << "normal_z: " << normal_z << std::endl;
             std::cout << "normal: " << normal << std::endl;
             std::cout << "cos_a: " << cos_a << std::endl;
             std::cout << "a: " << a << std::endl;
             std::cout << orientation_x << std::endl;*/

             poseMsg.header.stamp = ros::Time::now();            
             pcl_pub.publish(poseMsg); 
             ros::spinOnce();           
             }                  
    }
   
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_r(new pcl::PointCloud<pcl::PointXYZRGBA>);
   //createCloud(depth_res, color, pcloud_res);//segmentation fault
  
       
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Segmented Cloud Viewer"));
    const std::string cloudName = "rendered";
    //visualizer->addPointCloud(pcloud_res, cloudName);
    visualizer->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(pcloud_res,normals, 10, 0.03, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    while (!visualizer->wasStopped())
    {
       visualizer->spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    visualizer->close();
  } 




  void cloudViewer()
  {
    cv::Mat color, depth;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;   //处理时间
    //pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
//
  double fps = 0;
  size_t frameCount = 0;
  std::ostringstream oss;
  std::ostringstream ossXYZ; // 新增一个string流
  const cv::Point pos(5, 15);
  const cv::Scalar colorText = CV_RGB(255, 0, 0);
  const double sizeText = 0.5;
  const int lineText = 1;
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  // 从全局变量获取当前鼠标坐标
  int img_x = mouseX;
  int img_y = mouseY;
  
  geometry_msgs::PointStamped ptMsg;
  ptMsg.header.frame_id = "kinect2_link";
//
    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();
//
  const std::string window_name = "color viewer";
  cv::namedWindow(window_name);
  // 注册鼠标回调函数, 第三个参数是C++11中的关键字, 若不支持C++11, 替换成NULL
  cv::setMouseCallback(window_name, onMouse, nullptr);
//


    createCloud(depth, color, cloud);

    /*visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);*/

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

//
      img_x = mouseX;
      img_y = mouseY;
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_res(new pcl::PointCloud<pcl::PointXYZRGBA>);
      const pcl::PointXYZRGBA& pt = cloud->points[img_y * depth.cols + img_x];
     
      //ptMsg.point.x = img_x;
      //ptMsg.point.y = img_y;
      ptMsg.point.x = pt.x;
      ptMsg.point.y = pt.y;
      ptMsg.point.z = pt.z;
      
      
      // 根据鼠标左键压下或右键压下, 分别发布三维坐标到不同的话题上去
      switch (mouseBtnType) 
     {
      default:
          break;

      case cv::EVENT_RBUTTONUP:
          ptMsg.header.stamp = ros::Time::now();
          rightBtnPointPub.publish(ptMsg);
          ros::spinOnce();
          break;

      case cv::EVENT_LBUTTONUP:
          ptMsg.header.stamp = ros::Time::now();
          leftBtnPointPub.publish(ptMsg);
          ros::spinOnce();
          regionGrowingNeck(depth, color, cloud, img_x, img_y);
          
          //种子点区域增长
          //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pRegionGrowing;
          //pRegionGrowing = regionGrowing(depth, cloud, img_x, img_y);
          //regionGrowing1(depth, color, cloud, img_x, img_y);
                    
          break;
      }
      mouseBtnType = cv::EVENT_MOUSEMOVE;
     

      ++frameCount;
      now = std::chrono::high_resolution_clock::now();
      double elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
      if(elapsed >= 1.0)
     {
        fps = frameCount / elapsed;
        oss.str("");
        oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
        start = now;
        frameCount = 0;
      }

      cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
      ossXYZ.str("");
      ossXYZ << "( " << ptMsg.point.x << ", " << ptMsg.point.y
                                  << ", " << ptMsg.point.z << " )";
      cv::putText(color, ossXYZ.str(), cv::Point(img_x, img_y), font, 1, colorText, 3, CV_AA);
      // cv::circle(color, cv::Point(mouseX, mouseY), 5, cv::Scalar(0, 0, 255), -1);
      cv::imshow(window_name, color);
      // cv::imshow(window_name, depth);
      cv::waitKey(1);


      //visualizer->updatePointCloud(cloud, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      //visualizer->spinOnce(10);
    }
    //visualizer->close();
//
  cv::destroyAllWindows();
  cv::waitKey(100);
//
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];//itP cloud行指针
      const uint16_t *itD = depth.ptr<uint16_t>(r);//depth 行指针
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
         // itP->x = c; 
          //itP->y = r;
          //itP->z = 0;
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          //itP->r = itP->g = itP->b = itP->a = 0;
        //itP->b = itC->val[0];
        //itP->g = itC->val[1];
        //itP->r = itC->val[2];
       // itP->a = 255;

          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        //itP->x = c;
        //itP->y = r;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

void createCloudRow(const cv::Mat &depth,const int mouse_y,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const

  {
     
      pcl::PointXYZ *itP1 = &cloud->points[mouse_y * depth.cols];//itP cloud行指针 
      const uint16_t *itD1 = depth.ptr<uint16_t>(mouse_y);//depth 行指针
      const float y = lookupY.at<float>(0, mouse_y);
      const float *itX1 = lookupX.ptr<float>();
     // #pragma omp parallel 
      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP1, ++itD1, ++itX1)
      {
        register const float depthValue1 = *itD1 / 1000.0f;
        itP1->z = depthValue1;
        itP1->x = *itX1 * depthValue1;
        itP1->y = y * depthValue1;
      }

    }






  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving cloud: " << cloudName);
    writer.writeBinary(cloudName, *cloud);
    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  //ros::init(argc, argv, "kinect2_viewer_click", ros::init_options::AnonymousName);
  ros::init(argc, argv, "kinect2_viewer_click");

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
