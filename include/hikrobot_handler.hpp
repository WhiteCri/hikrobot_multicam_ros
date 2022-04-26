#pragma once

// highly refer https://github.com/luckyluckydadada/HIKROBOT-MVS-ROS-package/blob/main/README.md
// Since above one support only RGB 1 cam and I want mono8 multiple cam,
// I wrote this pkg

/* 
	TODO: publish camera info

    how to use this pkg 

    0. configure width, height, datatype(mono8, rgb, ...) using windows
        make sure set device name
    1. connect camera to linux with this pkg with Gethernet. Check if name set in 0 is shown by rosrun
    2. config done!
*/


#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <image_transport/image_transport.h>


//helper
template <typename T>
void getParam(ros::NodeHandle& nh, std::string name, T& v){
    if (false == nh.getParam(name, v)){
        std::string err_msg =  std::string() + 
            "param " + name + " does not exist!";
        throw std::runtime_error(err_msg.c_str());
    }
}

void error(std::string msg){
    ROS_ERROR("%s fail!", msg.c_str());
    exit(-1);
}

struct CameraProperty{
    int MAX_IMAGE_DATA_SIZE;
    int width;
    int height;
    std::string device_name;
    void init(){
        MAX_IMAGE_DATA_SIZE = width * height;
    }
};

class Camera{
    
public: //public functions
    Camera(ros::NodeHandle& nh, std::string cam_name);
    ~Camera();

    static void PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
    void ReadImg(cv::Mat &image);
    bool isValid();

    static int enumDevices();

private: //private functions
    void workThread();

    
private: //member variables
    std::mutex mtx;
    CameraProperty cam_config;
    void* camera_handle;
    std::thread thr;
    int run;
	image_transport::Publisher img_pub;
};

int Camera::enumDevices(){ //return number of devices that are found
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    
    if (MV_OK != nRet) error("MV_CC_EnumDevices()");

    if (stDeviceList.nDeviceNum == 0){
        ROS_WARN("No MV_CC devices found!");
        return 0;
    }

    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo) break;

        ROS_INFO("[device %d]: ", i);
        PrintDeviceInfo(pDeviceInfo);
    }
    return stDeviceList.nDeviceNum;
}

void Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo){
    if (NULL == pstMVDevInfo) {
        ROS_INFO("empty devinfo pointer");
        return;
    }

    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        struct in_addr ip_addr;
        ip_addr.s_addr = pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        ROS_INFO("nCurrentIp: %s", inet_ntoa(ip_addr));
        ROS_INFO("chUserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); 
    }
    // else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    // {
    //     printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    // }
    else
    {
        printf("Not support.\n");
    }
}

Camera::Camera(ros::NodeHandle& nh, std::string cam_name)
    : camera_handle(nullptr), run(0)
{
    getParam(nh, cam_name + "/width"        ,cam_config.width);
    getParam(nh, cam_name + "/height"       ,cam_config.height);
    getParam(nh, cam_name + "/device_name"  ,cam_config.device_name);
    cam_config.init();

    // create handle with device name
    int ret;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    if (MV_OK != nRet) error("MV_CC_EnumDevices");

    bool found = false;
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo) break;

        //copy string
        int buf_idx = 0;
        static char buf[1024] = "";
        unsigned char* p =pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
        while(*p) buf[buf_idx++] = *p++;

        found = cam_config.device_name == buf;
        if (found){
            ret = MV_CC_CreateHandle(&camera_handle, pDeviceInfo);
            if (MV_OK != ret) error("MV_CC_CreateHandle");

            ret = MV_CC_OpenDevice(camera_handle);
            if (MV_OK != ret) error("MV_CC_OpenDevice");

            break;
        }
    }
    if (false == found) {
		ROS_WARN("device %s not found", cam_config.device_name.c_str());
		return;
	}

    MVCC_ENUMVALUE t = {0};
    ret = MV_CC_GetEnumValue(camera_handle, "PixelFormat", &t);
    if (MV_OK == ret){
        if (t.nCurValue == PixelType_Gvsp_Mono8) ROS_INFO("Pixeltype: mono8");
        else ROS_INFO("not supported datatype");
    }

    // start frame capture thread
    ret = MV_CC_StartGrabbing(camera_handle);
    if (MV_OK != ret) error("MV_CC_StartGrabbing");

    run = 1;

	image_transport::ImageTransport it(nh);
	img_pub = it.advertise(cam_config.device_name + "/image_raw", 1);

    thr = std::thread(&Camera::workThread, this);
	ROS_WARN("created camera %s resource!", cam_config.device_name.c_str());
}

bool Camera::isValid(){
	return run == 1;
}

Camera::~Camera()
{
	if (camera_handle == nullptr) 
		return;
	
	if (run != -1){ // thread is running
    	run = 0;
    	while(run != -1) {
        	ROS_INFO("clean camera capture thread...");
        	usleep(500000); //0.5 second
    	}
	}
	thr.join();

    int ret;
    ret = MV_CC_StopGrabbing(camera_handle);
    if (MV_OK != ret) error("MV_CC_StopGrabbing");
    
    ret = MV_CC_CloseDevice(camera_handle);
    if (MV_OK != ret) error("MV_CC_CloseDevice");

    ret = MV_CC_DestroyHandle(camera_handle);
    if (MV_OK != ret) error("MV_CC_DestroyHandle");

	ROS_WARN("cleaned up camera %s resource!", cam_config.device_name.c_str());
}

void Camera::workThread(){
	int ret;

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    ret = MV_CC_GetIntValue(camera_handle, "PayloadSize", &stParam);
	if (MV_OK != ret) error("MV_CC_GetIntValue");
	unsigned long n_data_size = stParam.nCurValue;
	if (n_data_size != cam_config.height * cam_config.width) error("n_data_size != cam_config.height * cam_config.width");

	unsigned char *p_data = (unsigned char *)malloc(sizeof(unsigned char) * n_data_size);
	if (nullptr == p_data) error("malloc");

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
	cv::Mat last_frame;

	sensor_msgs::ImagePtr msg;
	while(run){
        ret = MV_CC_GetOneFrameTimeout(camera_handle, p_data, n_data_size, &stImageInfo, 200); // 200ms?
		auto start = std::chrono::steady_clock::now();
  		if (MV_OK != ret) error("MV_CC_GetOneFrameTimeout");

		last_frame = cv::Mat(cam_config.height, cam_config.width, CV_8UC1, p_data);
		msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", last_frame).toImageMsg();
		img_pub.publish(msg);
		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		ROS_INFO("driver ~ ros pub takes %lf seconds", elapsed/1000000.0);
    }
	run = -1;
}
