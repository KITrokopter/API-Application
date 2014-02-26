#include "APIQuadcopter.hpp"
#include <sstream>

using namespace kitrokopter;

void APIQuadcopter::APIQuadcopter(int id) {
    this->id = id;
    std::stringstream sstm;
    sstm << "api_quadcopter_" << id;
    ros::init(argc, argv, sstm.str());
    //empty the stringstream
    sstm.str("");
    sstm << "quadcopter_status_" << id;
    ros::Subscriber sub = nh.subscribe(sstm.str(), 1, statusCallback);  
}

/**
 * Get all channels where a quadcopter is online.
 *
 * @return array of channels
 */ 
uint8[] scanChannels() {
    std::stringstream sstm;
    sstm << "open_link_" << id;
    ros::ServiceClient client = this->nodeHandle.serviceClient<quadcopter_application::open_link>(sstm.str());
    quadcopter_application::open_link srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.channel = this->channel();
    if (!client.call(srv) || srv.response.error != 0) {
	ROS_ERROR("Failed to connect on channel.");
	return false;
    } else {
	return true;
    }
}

/**
 * Connect to a quadcopter on the given channel.
 * 
 * @param channel the channel to connect on
 * @return whether the connection attempt was successfully
 */ 
bool APIQuadcopter::connectOnChannel(uint8 channel) {
    this->channel = channel;
    std::stringstream sstm;
    sstm << "open_link_" << id;
    ros::ServiceClient client = this->nodeHandle.serviceClient<quadcopter_application::open_link>(sstm.str());
    quadcopter_application::open_link srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.channel = this->channel();
    if (!client.call(srv) || srv.response.error != 0) {
	ROS_ERROR("Failed to connect on channel.");
	return false;
    } else {
	return true;
    }
}

int APIQuadcopter::getId() {
    return this→id;
}

/**
 * Returns the channel on which the corresponding quadcopter module shall connect.
 * 
 * @return the channel
 */
uint8 APIQuadcopter::getChannel() {
    return this→channel();
}

/**
 * Performs a short start of the motors to be able to identify the quadcopter.
 */
void APIQuadcopter::blink() {
    std::stringstream sstm;
    sstm << "blink_" << id;
    ros::ServiceClient client = this->nodeHandle.serviceClient<quadcopter_application::blink>(sstm.str());
    quadcopter_application::blink srv;
    srv.request.header.stamp = ros::Time::now();
    if (!client.call(srv) || srv.response.error != 0) {
	ROS_ERROR("Failed to blink.");
	return 1;
    }
}

void APIQuadcopter::statusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg) {
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

