#include "GthTofApi.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <helper.h>

//add by yaoli 
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//ros
#include <ros/ros.h>

#define PCL_VISUALIZER 1
#if PCL_VISUALIZER
//visualization
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#endif


#define OPENCV_VISUALIZER 0
#ifdef OPENCV_VISUALIZER
#include <opencv/cv.hpp>
#endif


#ifdef OPENCV_VISUALIZER
void onMouse_depth(int event, int x, int y, int flags, void *param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
    std::cout << "depth(" << x << "," << y << ") :" << im->at<float>(cv::Point(x, y))<< "m" <<std::endl;
}

void onMouse_amplitude(int event, int x, int y, int flags, void *param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
    std::cout << "amplitude(" << x << "," << y << ") :" << static_cast<unsigned int>(im->at<int16_t>(cv::Point(x, y))) << std::endl;
}
#endif

typedef std::array<AlgorithmOutput_F32, 10> AlgorithmOutputArray;


int main(int argc, char** argv)
{
   ros::init (argc, argv, "main_node");
   ros::NodeHandle nh;
   //add example 
     std::string string_config;
    if (nh.getParam("start", string_config))
        ROS_INFO("start: %s", string_config.c_str());
    else
        ROS_WARN("No config name message");
 
//    double noise;
//    nh.getParam("noise", noise);
//    ROS_INFO("noise parameter is................... %f", noise);


   ros::Publisher tof_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1,true);
   ros::Rate loop_rate(20);

    Gth_LogConfig(GTH_LOG_LEVEL_INFO);

    Gth_Dev_Info dev;
    dev.type = Dev_Eth;
    dev.Info.eth.addr = "192.168.1.6";
    dev.Info.eth.port = 8567;
    dev.frameReady = NULL; //callback function

    Gth_Dev_Handle handle;
    if(0 != Gth_OpenDevice(&dev, &handle))
    {
        printf("open device failed!\n");
        return -1;
    }

   int  Range_Mode;
   nh.getParam("Range_Mode", Range_Mode);
   ROS_INFO("Range_Mode is...%d", Range_Mode);
   switch (Range_Mode)
{
    case 0:  Gth_SetRangeMode(&handle, Mode_Range_None);
    case 1:  Gth_SetRangeMode(&handle, Mode_Range_S);
    case 2:  Gth_SetRangeMode(&handle, Mode_Range_M);
    case 3:  Gth_SetRangeMode(&handle, Mode_Range_L);
    case 4:  Gth_SetRangeMode(&handle, Mode_Range_XL);
    case 5:  Gth_SetRangeMode(&handle, Mode_Range_Custom);
    case 6:  Gth_SetRangeMode(&handle, Mode_Range_WDR);
    default:   Gth_SetRangeMode(&handle, Mode_Range_S);
}

   int  FrameMode;
   nh.getParam("Range_Mode", FrameMode);
   ROS_INFO("Range_Mode is...%d", FrameMode);
   switch (FrameMode)
{
    case 0:  Gth_SetFrameMode(&handle, Mode_DistAmp);
    case 1:  Gth_SetFrameMode(&handle, Mode_RawPhases);
    case 2:  Gth_SetFrameMode(&handle, Mode_None);
    default:   Gth_SetFrameMode(&handle, Mode_RawPhases);
}

   int  minDepthRange,maxDepthRange;
   nh.getParam("minDepthRange", minDepthRange);
   ROS_INFO("minDepthRange is...%d", minDepthRange);
   nh.getParam("maxDepthRange", maxDepthRange);
   ROS_INFO("maxDepthRange is...%d", maxDepthRange);
   Gth_SetDepthRange(&handle, minDepthRange, maxDepthRange);

   int  AmplitudeThreshold;
   nh.getParam("AmplitudeThreshold", AmplitudeThreshold);
   ROS_INFO("AmplitudeThreshold is...%d", AmplitudeThreshold);
   Gth_SetAmplitudeThreshold(&handle, AmplitudeThreshold);

   int  SetIntegrationTime;
   nh.getParam("SetIntegrationTime", SetIntegrationTime);
   ROS_INFO("SetIntegrationTime is...%d", SetIntegrationTime);
   Gth_SetIntegrationTime(&handle, SetIntegrationTime);

   int  FrameRate;
   nh.getParam("FrameRate", FrameRate);
   ROS_INFO("FrameRate is...%d", FrameRate);
   Gth_SetFrameRate(&handle, FrameRate);

   int  ModulationFrequency;
   nh.getParam("ModulationFrequency", ModulationFrequency);
   ROS_INFO("ModulationFrequency is...%d", ModulationFrequency);
   Gth_SetModulationFrequency(&handle, ModulationFrequency);
    
  int  Gain;
   nh.getParam("Gain", Gain);
   ROS_INFO("Gain is...%d", Gain);
   Gth_SetGain(&handle, Gain);
    
   int  FlipMirror;
   nh.getParam("FlipMirror", FlipMirror);
   ROS_INFO("FlipMirror is...%d", FlipMirror);
   switch (FlipMirror)
{
    case 0:  Gth_SetFlipMirror(&handle, Default);
    case 1:  Gth_SetFlipMirror(&handle, Flip);
    case 2:  Gth_SetFlipMirror(&handle, Mirror);
    case 3:  Gth_SetFlipMirror(&handle, Flip_Mirror);
    default:   Gth_SetFlipMirror(&handle, Default);
}


    //Gth_SetDepthRange(&handle, 0, 5000);

    Gth_StartStream(&handle);

    AlgorithmOutputArray output_data;
    int output_idx = 0;
    Gth_Frame frame;
    uint8_t* depth_rgb = new uint8_t[IMAGE_HEIGHT*IMAGE_WIDTH*3];
    uint8_t* dst_ir = new uint8_t[IMAGE_HEIGHT*IMAGE_WIDTH];
    float* pcl = new float[IMAGE_HEIGHT*IMAGE_WIDTH*3];

#ifdef PCL_VISUALIZER
    /****** pcl viewer******/
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem (0.3, 0.3, 0.3, 0.3);
    viewer->initCameraParameters();
    float theta = M_PI; // The angle of rotation in radians
    transform (0,0) = cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = cos (theta);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
#endif

    while (ros::ok())
    {
       // TOF_Frame frame;
        if(0 == Gth_GetFrame(&handle, &frame))
        {
            printf("frame index:%d\n", frame.index);
            Gth_GetDepthF32andAmplitudeData(&handle,
                                            &frame,
                                            output_data[output_idx].depth.get(),
                                            output_data[output_idx].amplitude.get());

            /* then you can process depth data and amplitude data */
#ifdef OPENCV_VISUALIZER
            cv::Mat tof_depth = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32F, output_data[output_idx].depth.get()); //原始深度图
            cv::Mat tof_amplitude= cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16S, output_data[output_idx].amplitude.get()); //原始幅度图

            /*** In order to obtain higher quality images, bilateral filtering is recommended. ***/
//            cv::Mat img_tof_depth_filter;
//            cv::bilateralFilter(tof_depth*1000, img_tof_depth_filter, 20, 40, 10);
//            cv::Mat tof_depth_f =img_tof_depth_filter/1000;
  //          tof_depth_f.copyTo(tof_depth);
#endif
            /* decode one depth_f32 data to rgb */
            Gth_DepthF32ToRGB(&handle, depth_rgb, IMAGE_HEIGHT*IMAGE_WIDTH*3, output_data[output_idx].depth.get(), IMAGE_HEIGHT*IMAGE_WIDTH, 0, 3.747);

            /* decode one amplitude data to gray */
            Gth_AmplitudeToIR(&handle, dst_ir, IMAGE_HEIGHT*IMAGE_WIDTH, output_data[output_idx].amplitude.get(), IMAGE_HEIGHT*IMAGE_WIDTH, 1200);

#ifdef OPENCV_VISUALIZER
            cv::Mat tof_depth_RGB = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, depth_rgb); //渲染后的深度图
            cv::Mat tof_amplitude_IR = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, dst_ir);//渲染后的幅度图

            cv::namedWindow("depth", 0);
    //        cv::setMouseCallback("depth", onMouse_depth, reinterpret_cast<void *>(&tof_depth));
            cv::imshow("depth", tof_depth_RGB);
            cv::waitKey(1);

            cv::namedWindow("amplitude", 0);
      //      cv::setMouseCallback("amplitude", onMouse_amplitude, reinterpret_cast<void *>(&tof_amplitude));
            cv::imshow("amplitude", tof_amplitude_IR);
            cv::waitKey(1);
#endif
            /*** Get point cloud data from distance data. ***/
           Gth_GetXYZDataF32_f(&handle, output_data[output_idx].depth.get(), pcl, IMAGE_HEIGHT*IMAGE_WIDTH);

#ifdef PCL_VISUALIZER
            viewer->removeAllPointClouds();
            point_cloud_ptr->clear();

            for(int i=0;i<IMAGE_HEIGHT;i++)
            {
                for(int j=0;j<IMAGE_WIDTH;j++)
                {
                    int index = i*IMAGE_WIDTH+j;
                    pcl::PointXYZ point;
                    point.x = pcl[index*3+0];
                    point.y = pcl[index*3+1];
                    point.z = pcl[index*3+2];

                    if(point.z > 0)
                    {
                        point_cloud_ptr->points.push_back(point);
                    }
                }
            }
            point_cloud_ptr->width = point_cloud_ptr->points.size();
            point_cloud_ptr->height = 1;

              //ros viewer is needed add add by yaoli 2020.07.11.23:53 
        point_cloud_ptr->is_dense = true;
        point_cloud_ptr->header.seq=0;//need setting
        point_cloud_ptr->header.stamp=0;//need setting
        point_cloud_ptr->header.frame_id="map";//need setting


        sensor_msgs::PointCloud2 output;   
        //pcl_conversions::fromPCL(cloud_temp, output); 
        pcl::toROSMsg(*point_cloud_ptr,output);
        // output->header = point_cloud_ptr->header
        tof_pub.publish(output);



       // cout<<"point_cloud_ptr->points.size() : "<<point_cloud_ptr->points.size()<<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*point_cloud_ptr, *transformed_cloud, transform);
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(point_cloud_ptr, "z");//按照z字段进行渲染
        viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, fildColor);//显示点云，其中fildColor为颜色显示
        viewer->spinOnce(1);
        boost::this_thread::sleep(boost::posix_time::microseconds(1));
#endif

            Gth_FreeFrame(&handle, &frame);
            output_idx = output_idx < 9 ? output_idx + 1 : 0;
        }
    }
    
    loop_rate.sleep();
    Gth_StopStream(&handle);
    Gth_CloseDevice(&handle);

    delete[] depth_rgb;
    delete[] dst_ir;
    delete[] pcl;
    return 0;
}


