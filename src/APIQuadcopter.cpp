#include "APIQuadcopter.hpp"
#include <sstream>

using namespace kitrokopter;

void APIQuadcopter::APIQuadcopter(int newid) {
    this->id = newid;
    this->selectedForFlight = true;
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
    sstm << "search_links_" << id;
    ros::ServiceClient client = this->nodeHandle.serviceClient<quadcopter_application::search_links>(sstm.str());
    quadcopter_application::search_links srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.channel = this->channel();
    if (!client.call(srv)) {
	ROS_ERROR("Failed to scan channels.");
	return 
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

/**
 * Get the id of the corresponding quadcopter module.
 *
 * @return the id
 */ 
int APIQuadcopter::getId() {
    return this->id;
}

/**
 * Returns the channel on which the corresponding quadcopter module shall connect.
 * 
 * @return the channel
 */
uint8 APIQuadcopter::getChannel() {
    return this->channel();
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

/**
 * Set the color range for tracking this quadcopter.
 * 
 * @param min the lower bound
 * @param max the upper bound
 */
void APIQuadcopter::setColorRange(uint32_t min, uint32_t max) {
    this->colorRange = {min, max};
}

/**
 * Set the color range for tracking this quadcopter.
 * 
 * @param range[2] lower bound, upper bound
 */
void APIQuadcopter::setColorRange(uint32_t range[2]) {
    this->colorRange = range;
}

/**
 * Get the color range for tracking this quadcopter.
 * 
 * @return the color range
 */
uint32_t[2] APIQuadcopter::getColorRange() {
    return this->colorRange;
}

/**
 * Whether this quadcopter is selected to fly or to stay on the ground.
 * 
 * @return whether this quadcopter shall fly
 */
bool APIQuadcopter::isSelectedForFlight() {
    return this->selectedForFlight;
}

/**
 * Set whether this quadcopter shall fly or stay on the ground.
 * 
 * @param select whether this quadcopter shall fly or stay on the ground
 */
void APIQuadcopter::setSelectedForFlight(bool select) {
   this->selectedForFlight = select; 
}

/**
 * Get the quality of the connection to the quadcopter.
 * 
 * @return the link quality
 */
float getLinkQuality() {
    return this->linkQuality;
}

/**
 * Get the current acceleration.
 * 
 * @return current acceleration
 */
float getCurrentAcceleration() {
    return (this->currentSpeedValues[1] - this->currentSpeedValues[0]) / (this->currentSpeedTimestamps[1] - this->currentSpeedTimestamps[0]);
}

/**
 * Get the current acceleration.
 * 
 * @return current acceleration
 */
float getCurrentSpeed() {
    //return the newest current speed value
    return this->currentSpeedValues[1];
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

