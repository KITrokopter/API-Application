#include "API.hpp"
#include "ros/ros.h"
#include <ros/console.h>

using namespace kitrokopter;

API::API(int argc, char **argv)
{
    idCounter = 0;
    
    cameras = vector<int[2]>(8);
    quadcopters = vector<int>(10);
    controllers = vector<int>(1);
    positions = vector<int>(5);
    
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
	    cameras.push_back({res.id, req.camera_id});
	    break;
	case 1:
	    quadcopters.push_back(res.id);
	    break;
	case 2:
	    controllers.push_back(res.id);
	    break;
	case 3:
	    positions.push_back(res.id);
	    break;
	default:
	    ROS_ERROR("Malformed register attempt!");
	    res.id = -1;
	    return false;
    }
    ROS_INFO("Registered new module with type %d and id %d", req.type, res.id);
    return true;
}