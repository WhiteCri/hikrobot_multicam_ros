#include <ros/ros.h>
#include "signal.h" 
#include "hikrobot_handler.hpp"

static int run = 1;

void myhandler(int signo){
	run = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hikrobot_camera_driver");
    ros::NodeHandle nh;

   	signal(SIGINT, myhandler);

    int n_device = Camera::enumDevices();
    if (n_device == 0) {
        ROS_WARN("no camera detected");
        exit(-1);
    }
    
    std::string cam1_name, cam2_name;
    if (false == nh.getParam("cam1_name", cam1_name))
        throw std::runtime_error("cam1_name is not set!");
    if (false == nh.getParam("cam2_name", cam2_name))
        throw std::runtime_error("cam2_name is not set!");
    
    Camera* cam1 = new Camera(nh, cam1_name);
    Camera* cam2 = new Camera(nh, cam2_name);
    
	while(run){
		sleep(1);
	}

    delete cam1;
    delete cam2;
    std::cout << "cam1, cam2 deleted" << std::endl;
}
