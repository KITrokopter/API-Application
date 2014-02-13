#include "API.hpp"
#include "ros/ros.h"
#include <ros/console.h>

using namespace kitrokopter;

API::API(int argc, char **argv)
{
    this->idCounter = 0;
    
    this->cameras = std::vector<std::pair<int, long> >(8);
    this->quadcopters = std::vector<int>(10);
    this->controllers = std::vector<int>(1);
    this->positions = std::vector<int>(5);
    
    ros::init(argc, argv, "api_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("announce", &API::announce, this);
    ROS_INFO("Ready to deliver IDs.");
    ros::spin();
}

bool API::announce(api_application::Announce::Request &req, api_application::Announce::Response &res)
{
    res.id = idCounter++;
    switch (req.type) {
	case 0:
	    this->cameras.push_back({res.id, req.camera_id});
	    break;
	case 1:
	    this->quadcopters.push_back(res.id);
	    break;
	case 2:
	    this->controllers.push_back(res.id);
	    break;
	case 3:
	    this->positions.push_back(res.id);
	    break;
	default:
	    ROS_ERROR("Malformed register attempt!");
	    res.id = -1;
	    return false;
    }
    ROS_INFO("Registered new module with type %d and id %d", req.type, res.id);
    return true;
}

int main(int argc, char** argv)
{
    API api = new API(argc, argv);
    ros::shutdown();
    
    // Wait for ros to shutdown.
    while (ros::ok()) {
	usleep(10000);
    }
    
    std::cout << "API Application successfully terminated" << std::endl;
}    