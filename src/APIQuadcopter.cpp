#include "APIQuadcopter.hpp"
#include <sstream>

using namespace kitrokopter;

APIQuadcopter::APIQuadcopter(int id) {
    this->id = id;
    std::stringstream sstm;
    sstm << "api_quadcopter_" << id;
    ros::init(argc, argv, sstm.str());
    //empty the stringstream
    sstm.str("");
    sstm << "quadcopter_status_" << id;
    ros::Subscriber sub = nh.subscribe(sstm.str(), 1, statusCallback);  
}

APIQuadcopter::connectOnChannel(uint8 channel) {
    this->channel = channel;
    std::stringstream sstm;
    sstm << "api_quadcopter_" << id;
    ros::ServiceClient client = this->nodeHandle.serviceClient<quadcopter_application::open_link>("open_link");
    quadcopter_application::open_link srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.channel = this->channel();
    if (!client.call(srv) || srv.response.error != 0) {
	ROS_ERROR("Failed to connect on channel.");
	return 1;
    }
}

APIQuadcopter::statusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg) {
    ROS_INFO("Got new data (linkquality, roll, pitch, yaw): %f, %f, %f, %f",
	     msg->link_quality,
	     msg->stabilizer_roll
	     msg->stabilizer_pitch
	     msg->stabilizer_yaw
	    );
    this->linkQuality = msg->link_quality;
    this->stabilizerRollData = msg->stabilizer_roll;
    this->stabilizerPitchData = msg->stabilizer_pitch;
    this->stabilizerYawData = msg->stabilizer_yaw;
}

