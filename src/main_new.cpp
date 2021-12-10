#include "Tofsdk.h"
#include <opencv/cv.hpp>
#include <opencv/cv.h>

#include <iostream>

#include <stdio.h>
#include <fstream>
#include <helper.h>
#include <unistd.h>
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

void writeMatToFile(cv::Mat& m, const char* filename);


void writeMatToFile(cv::Mat& m, const char* filename)
{
	std::ofstream fout(filename);
	if (!fout)
	{
		std::cout << "File Not Opened" << std::endl;
		return;
	}
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			fout << m.at<float>(i, j) << "\t";
		}
		fout << std::endl;
	}
	fout.close();
}


void onMouse_depth(int event, int x, int y, int flags, void *param)
{
	cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
	std::cout << "depth(" << x << "," << y << ") :" << static_cast<unsigned int>(im->at<float>(cv::Point(x, y))) << std::endl;
}

void onMouse_amplitude(int event, int x, int y, int flags, void *param)
{
	cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
	std::cout << "amplitude(" << x << "," << y << ") :" << static_cast<unsigned int>(im->at<float>(cv::Point(x, y))) << std::endl;
}

typedef std::array<AlgorithmOutput, 10> AlgorithmOutputArray;
typedef std::array<PhaseOutput, 10> PhaseOutputArray;

float f = 3.3;//焦距，单位mm
float pixel_size = 0.015;//像元尺寸，单位mm
float fx_reciprocal = pixel_size / f;
/*** 深度图转换成PCL点云计算函数 ***/
/*
void depthToPointCloud(cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_ptr)
{
    for (int r = 0; r < 240; ++r)
    {
        for (int c = 0; c < 288; ++c)
        {
            pcl::PointXYZ point;
            int depth = depth_image.ptr<float>(r)[c];
            float delta_x = (c + 1 - 120) * fx_reciprocal;
            float delta_y = (r + 1 - 144) * fx_reciprocal;

            point.z = depth / (std::sqrt(delta_x*delta_x + delta_y*delta_y + 1)) / 1000;
            point.x = delta_x * point.z;
            point.y = delta_y * point.z;

            if (point.z > 0 && point.z < 3)
            {
                point_cloud_ptr->points.push_back(point);
            }
        }
    }
    point_cloud_ptr->width = point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
}
*/

int main(int argc, char** argv)
{
   ros::init (argc, argv, "main_node");
   ros::NodeHandle nh;
   //add example 
    //  std::string string_config;
    // if (nh.getParam("start", string_config))
    //     ROS_INFO("start: %s", string_config.c_str());
    // else
    //     ROS_WARN("No config name message");
 
//    double noise;
//    nh.getParam("noise", noise);
//    ROS_INFO("noise parameter is................... %f", noise);


 	GTH::ITofSdk* tof = GTH::ITofSdk::create();


	TOF_Config config;
	config.tcpDeviceIpAddr = "192.168.1.6"; //相机平台Ip地址
	config.tcpDataPort = 8567;              //相机平台服务端端口号
	config.udpDataIpAddr = "192.168.1.6";
	config.udpDataPort = 8567;
	config.frameArrived = NULL;
	if (TOF_StatusOk != tof->open(config))
	{
		printf("open failed!\n");
		return -1;
	}


    TOF_Parameters para;
    tof->getTofParameters(para);

    /*************设置为S工作模式*************/
    para.frameMode = TOF_FrameModeRawPhases;
    para.integrationTime = 500;    //300
    para.gain = 1;
    para.modulationFrequency = 40;
    tof->setTofParameters(para);

     // /*************设置为M工作模式*************/
    // para.frameMode = TOF_FrameModeRawPhases;
    // para.integrationTime = 1200;   //500
    // para.gain = 4;                 //2
    // para.modulationFrequency = 40;
    // tof->setTofParameters(para);

    // /*************设置为L工作模式*************/
    // para.frameMode = TOF_FrameModeRawPhases;
    // para.integrationTime = 1200;
    // para.gain = 4;
    // para.modulationFrequency = 20; //40
    // tof->setTofParameters(para);

    // /*************设置为XL工作模式*************/
    // para.frameMode = TOF_FrameModeRawPhases;
    // para.integrationTime = 1500;   //1200
    // para.gain = 4;
    // para.modulationFrequency = 12;  //40
    // tof->setTofParameters(para);


    /*** 设置为长曝光模式,参数为800us、40MHz、4倍增益，有效测试距离为0.7m-2cm ****/
//    para.frameMode = TOF_FrameModeRawPhases;
//    para.integrationTime = 800;
//    para.gain = 4;
//    para.frameRate = 10;
//    tof->setTofParameters(para);

    /*** 设置为短曝光模式,参数为500us、40MHz、1倍增益，有效测试距离为0.3m-1.2cm ****/
//    para.frameMode = TOF_FrameModeRawPhases;
//    para.integrationTime = 500;
//    para.gain = 1;
//    tof->setTofParameters(para);

    /*** (默认模式)设置为长短曝光融合模式,由上述参数长短曝光2帧融合为一帧,有效测试距离为0.3m-2cm****/
//    para.frameMode = TOF_FrameModeLongShortExposure;
//    para.integrationTime = 800;
//    para.gain = 4;
//    para.integrationTime_short = 500;
//    para.gain_short = 1;
//    para.frameRate = 15;
//    para.modulationFrequency_short = 40;
//    tof->setTofParameters(para);

	/***设置幅度值阈值，默认为0 ***/
	//tof->setAmplitudeThreshold(0);

	/***是否开启滤波，默认开启 ***/
	//     tof->isFilter(false);

	/***设置全局误差，默认为0，单位为mm ***/
	//tof->setGlobalOffset(20);

	//RawPhases Mode
	AlgorithmOutputArray output_data;
	int output_idx = 0;
	PhaseOutputArray four_phase;

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
    	TOF_Frame frame;
		if (TOF_StatusOk == tof->getFrame(frame))//从下位机发到上位机的网络包中取得4个相位phase
		{
			if (frame.vesalTemp > 79)//判断相机当前的温度是否超过79度，如果超过，则输出警告
			{
				printf("Detected system getting too warm\n");
			}

            int16_t* phase1 = NULL;//创建4个相位数据，用来对TOF_FrameModeLongShortExposure和TOF_FrameModeRawphase模式进行深度图和幅度图的计算
            int16_t* phase2 = NULL;
            int16_t* phase3 = NULL;
            int16_t* phase4 = NULL;

            uint16_t* distance = NULL;//如果当前相机的模式TOF_FrameModeDistAmp，则用distance来接收下位机传上来的深度图数据，amplitude接收幅度图数据
            uint16_t* amplitude = NULL;

            cv::Mat img_tof_depth(240,288,CV_32F);
            cv::Mat img_tof_amplitude(240,288,CV_32F);      

            if(TOF_FrameModeDistAmp == frame.mode)
            {
                if(TOF_StatusOk == tof->getDistances(frame, (void*&)distance) && TOF_StatusOk == tof->getAmplitudes(frame, (void*&)amplitude))//接收深度图数据和幅度图数据
                {
                    cv::Mat tof_depth = cv::Mat(HEIGHT, WIDTH, CV_16U, distance);//将接收到的深度图数据和幅度图数据分别保存在tof_depth和tof_amplitude中
                    cv::Mat tof_amplitude = cv::Mat(HEIGHT, WIDTH, CV_16U, amplitude);

                    cv::Mat img_tof_depth1 = tof_depth(cv::Rect(14, 0, 288, 240));//对深度图进行区域分割，得到240*288的深度图
                    cv::Mat img_tof_amplitude1 = tof_amplitude(cv::Rect(14, 0, 288, 240));//对幅度图进行区域分割，得到240*288的深度图

                    //cv::Mat img_tof_depth,img_tof_amplitude;
                    img_tof_depth1.convertTo(img_tof_depth,CV_32F);
                    img_tof_amplitude1.convertTo(img_tof_amplitude,CV_32F);//这里将幅度图数据和深度图数据从CV_16U转换成了CV_32F，目的是为了做双边滤波，因为做双边滤波的前提是数据必须为CV_8U或者CV_32F
                }
            }

			if (TOF_FrameModeLongShortExposure == frame.mode)
            {
				if (TOF_StatusOk == tof->getPhases(frame, (void*&)phase1, (void*&)phase2, (void*&)phase3, (void*&)phase4))
				{

                    if (1 == frame.paraIndex) //short exposure
					{
						memcpy(four_phase[output_idx].phase1.get(), phase1, OUTPUT_BUFFER_SIZE);
						memcpy(four_phase[output_idx].phase2.get(), phase2, OUTPUT_BUFFER_SIZE);
						memcpy(four_phase[output_idx].phase3.get(), phase3, OUTPUT_BUFFER_SIZE);
						memcpy(four_phase[output_idx].phase4.get(), phase4, OUTPUT_BUFFER_SIZE);
					}
                    else if (0 == frame.paraIndex) //long exposure
					{
                        //长短曝光融合计算
						tof->calculateLongShortExposure(phase1, phase2, phase3, phase4,
							four_phase[output_idx].phase1.get(),
							four_phase[output_idx].phase2.get(),
							four_phase[output_idx].phase3.get(),
							four_phase[output_idx].phase4.get(),
							output_data[output_idx].depth.get(),
							output_data[output_idx].confidence.get());//进行长短曝光的计算

                        cv::Mat tof_depth = cv::Mat(HEIGHT, WIDTH, CV_32F, output_data[output_idx].depth.get()); //深度图
                        cv::Mat tof_amplitude = cv::Mat(HEIGHT, WIDTH, CV_32F, output_data[output_idx].confidence.get()); //幅度图

                        img_tof_depth = tof_depth(cv::Rect(14, 0, 288, 240));
                        img_tof_amplitude = tof_amplitude(cv::Rect(14, 0, 288, 240));
					}
				}
			}

            if (TOF_FrameModeRawPhases == frame.mode)
            {
                if (TOF_StatusOk == tof->getPhases(frame, (void*&)phase1, (void*&)phase2, (void*&)phase3, (void*&)phase4))
                {
                    if(800 == para.integrationTime && 4 == para.gain)//判断当前模式是否为长曝光
                    {
                        //长曝光计算
                        tof->calculateLongShortExposure_L(phase1, phase2, phase3, phase4,
                            output_data[output_idx].depth.get(),
                            output_data[output_idx].confidence.get());//计算长曝光模式下的深度图和幅度图
                    }
                    else
                    {
                        //短曝光计算
                        tof->calculateLongShortExposure_four_mode(phase1, phase2, phase3, phase4,
                            output_data[output_idx].depth.get(),
                            output_data[output_idx].confidence.get());//计算新加入的四种工作模式下的深度图和幅度图
                    }

                    cv::Mat tof_depth = cv::Mat(HEIGHT, WIDTH, CV_32F, output_data[output_idx].depth.get()); //深度图
                    cv::Mat tof_amplitude = cv::Mat(HEIGHT, WIDTH, CV_32F, output_data[output_idx].confidence.get()); //幅度图

                    img_tof_depth = tof_depth(cv::Rect(14, 0, 288, 240));//对深度图进行区域分割，得到240*288的深度图
                    img_tof_amplitude = tof_amplitude(cv::Rect(14, 0, 288, 240));//对幅度图进行区域分割，得到240*288的深度图
                }
            }

             //  /***中值滤波***/
            tof->medianFilter(img_tof_depth,3);//封装好的中值滤波函数，3是滤波核

            // /***sobel滤波***/
            tof->sobelFilter(img_tof_depth,3,1800,300); //封装好的sobel滤波函数，3是滤波核，1800是最大深度值，300是最小深度值

            // /***双边滤波***/
            tof->setbilateralFilter(img_tof_depth,10,15,15);//封装好的双边滤波函数，10是滤波半径，15，15分别是值域方差和空域方差
            
            //深度图渲染后显示
            auto diff0 = 3000 + std::numeric_limits<double>::epsilon();
            cv::Mat tof_copy = img_tof_depth;
            tof_copy.convertTo(tof_copy, CV_8U, 255 / diff0);//将tof_copy的像素值缩放到0~255区间
            cv::applyColorMap(tof_copy, tof_copy, cv::COLORMAP_RAINBOW);//调用opencv的渲染函数,cv::COLORMAP_RAINBOW表明渲染的颜色为红色到紫色的渐变色(渲染你结果是彩色图)
            //cv::applyColorMap(tof_copy, tof_copy, cv::COLORMAP_BONE);
            cv::namedWindow("depth", 0);//创建显示窗口，窗口名为depth
            cv::setMouseCallback("depth", onMouse_depth, reinterpret_cast<void *>(&img_tof_depth));//在渲染的深度图上，鼠标点击处的像素值对应深度图img_tof_depth的像素值

            cv::imshow("depth", tof_copy);//显示渲染后的深度图
            cv::waitKey(1);//显示的每一帧图像持续1ms

            //幅度图渲染后显示
            auto diff1 = 1500 + std::numeric_limits<double>::epsilon();
            cv::Mat confidence_copy = img_tof_amplitude;
            confidence_copy.convertTo(confidence_copy, CV_8U, 255 / diff1);//将confidence_copy的像素值缩放到0~255区间
            cv::applyColorMap(confidence_copy, confidence_copy, cv::COLORMAP_BONE);//调用opencv的渲染函数，cv::COLORMAP_BONE表示渲染的颜色为从黑到白的渐变色(渲染结果是黑白图)
            cv::namedWindow("amplitude", 0);//创建显示窗口，窗口名为amplitude
            cv::setMouseCallback("amplitude", onMouse_amplitude, reinterpret_cast<void *>(&img_tof_amplitude));//在渲染的幅度图上，鼠标点击处的像素值对应深度图img_tof_amplitude的像素值

            cv::imshow("amplitude", confidence_copy);//显示渲染后的深度图
            cv::waitKey(1);//显示的每一帧图像持续1ms
            output_idx = output_idx < 9 ? output_idx + 1 : 0;

            tof->freeFrame(frame);
		}
	}
    return 0;
}


// #ifdef PCL_VISUALIZER
//             viewer->removeAllPointClouds();
//             point_cloud_ptr->clear();

//             for(int i=0;i<IMAGE_HEIGHT;i++)
//             {
//                 for(int j=0;j<IMAGE_WIDTH;j++)
//                 {
//                     int index = i*IMAGE_WIDTH+j;
//                     pcl::PointXYZ point;
//                     point.x = pcl[index*3+0];
//                     point.y = pcl[index*3+1];
//                     point.z = pcl[index*3+2];

//                     if(point.z > 0)
//                     {
//                         point_cloud_ptr->points.push_back(point);
//                     }
//                 }
//             }
//             point_cloud_ptr->width = point_cloud_ptr->points.size();
//             point_cloud_ptr->height = 1;

//               //ros viewer is needed add add by yaoli 2020.07.11.23:53 
//         point_cloud_ptr->is_dense = true;
//         point_cloud_ptr->header.seq=0;//need setting
//         point_cloud_ptr->header.stamp=0;//need setting
//         point_cloud_ptr->header.frame_id="map";//need setting


//         sensor_msgs::PointCloud2 output;   
//         //pcl_conversions::fromPCL(cloud_temp, output); 
//         pcl::toROSMsg(*point_cloud_ptr,output);
//         // output->header = point_cloud_ptr->header
//         tof_pub.publish(output);



//        // cout<<"point_cloud_ptr->points.size() : "<<point_cloud_ptr->points.size()<<endl;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//         pcl::transformPointCloud(*point_cloud_ptr, *transformed_cloud, transform);
//         pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(point_cloud_ptr, "z");//按照z字段进行渲染
//         viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, fildColor);//显示点云，其中fildColor为颜色显示
//         viewer->spinOnce(1);
//         boost::this_thread::sleep(boost::posix_time::microseconds(1));
// #endif
