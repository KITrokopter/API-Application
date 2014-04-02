#include "APIQuadcopter.hpp"
#include "quadcopter_application/search_links.h"
#include "quadcopter_application/open_link.h"
#include "quadcopter_application/blink.h"
#include <sstream>
#include "ros/ros.h"

using namespace kitrokopter;

APIQuadcopter::APIQuadcopter(int newid) : 
    id(newid),
    selectedForFlight(true),
    channel(-1),
    colorRange{0, 0},
    currentSpeedValues{Vector(), Vector()},
    currentSpeedTimestamps{0, 0},
    currentAcceleration(Vector()),
    currentPositionValues{Vector(), Vector(-1.0, -1.0, -1.0)},
    currentPositionTimestamps{0, 0},
    currentOrientation(Vector()),
    linkQuality(0.0),
    targetSpeed(0.0),
    targetAcceleration(0.0),
    targetPosition(Vector()),
    targetOrientation(Vector()),
    stabilizerRollData(0.0),
    stabilizerPitchData(0.0),
    stabilizerYawData(0.0),
    batteryStatus(0.0)
{
}

/**
 * Starts listening to the Quadcopter's ROS topic.
 *
 * This method should only be called once.
 */
void APIQuadcopter::listen()
{
    std::stringstream sstm;
    sstm << "quadcopter_status_" << id;
    ros::NodeHandle nodeHandle;
    this->statusSubscriber = nodeHandle.subscribe(sstm.str(), 1,
                                               &APIQuadcopter::statusCallback, this);
    sstm.str("");
    sstm << "quadcopter_position_" << id;
    this->positionSubscriber = nodeHandle.subscribe(sstm.str(), 1,
                                     &APIQuadcopter::positionCallback, this);
}

/**
 * Get all channels where a quadcopter is online.
 *
 * @return array of channels
 */
std::vector<uint8_t> APIQuadcopter::scanChannels() {
    std::stringstream sstm;
    sstm << "search_links_" << id;
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient< quadcopter_application::search_links > (sstm.str());
    quadcopter_application::search_links srv;
    srv.request.header.stamp = ros::Time::now();
    if (!client.call(srv)) {
        ROS_ERROR("Failed to scan channels.");
        return std::vector<uint8_t>();
    } else {
        return srv.response.channels;
    }
}

/**
 * Connect to a quadcopter on the given channel.
 * 
 * @param channel the channel to connect on
 * @return whether the connection attempt was successfully
 */
bool APIQuadcopter::connectOnChannel(uint8_t channel) {
    this->channel = channel;
    std::stringstream sstm;
    sstm << "open_link_" << id;
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient
    < quadcopter_application::open_link > (sstm.str());
    quadcopter_application::open_link srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.channel = this->channel;
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
int APIQuadcopter::getChannel() {
    return this->channel;
}

/**
 * Performs a short start of the motors to be able to identify the quadcopter.
 */
void APIQuadcopter::blink() {
    std::stringstream sstm;
    sstm << "blink_" << id;
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<quadcopter_application::blink>(sstm.str());
    quadcopter_application::blink srv;
    srv.request.header.stamp = ros::Time::now();
    if (!client.call(srv) || srv.response.error != 0) {
        throw new std::runtime_error("unable to blink");
    }
}

/**
 * Set the color range for tracking this quadcopter.
 * 
 * @param min the lower bound
 * @param max the upper bound
 */
void APIQuadcopter::setColorRange(uint32_t min, uint32_t max) {
    colorRange[0] = min;
    colorRange[1] = max;
}

/**
 * Set the color range for tracking this quadcopter.
 * 
 * @param range[2] lower bound, upper bound
 */
void APIQuadcopter::setColorRange(uint32_t range[2]) {
    setColorRange(range[0], range[1]);
}

/**
 * Get the color range for tracking this quadcopter.
 * 
 * @return the color range
 */
uint32_t* APIQuadcopter::getColorRange() {
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
float APIQuadcopter::getLinkQuality() {
    return this->linkQuality;
}

/**
 * Get the battery voltage as indicator for the battery charging.
 * 
 * @return the battery voltage
 */
float APIQuadcopter::getBatteryStatus() {
    return this->batteryStatus;
}

/**
 * Get the current acceleration as a vector.
 * 
 * @return current acceleration in mm/s²
 */
Vector APIQuadcopter::getCurrentAcceleration() {
    if (this->currentSpeedTimestamps[1] - this->currentSpeedTimestamps[0] == 0)
    {
        return 0.0;
    } else {
        double x = (this->currentSpeedValues[1].getX() - this->currentSpeedValues[0].getX())
        / (this->currentSpeedTimestamps[1] - this->currentSpeedTimestamps[0]);
        double y = (this->currentSpeedValues[1].getY() - this->currentSpeedValues[0].getY())
        / (this->currentSpeedTimestamps[1] - this->currentSpeedTimestamps[0]);
        double z = (this->currentSpeedValues[1].getZ() - this->currentSpeedValues[0].getZ())
        / (this->currentSpeedTimestamps[1] - this->currentSpeedTimestamps[0]);
        return Vector(x, y, z);
        
    }
}

/**
 * Get the current speed as vector.
 * 
 * @return current speed in mm/s
 */
Vector APIQuadcopter::getCurrentSpeed() {
    //return the newest current speed value
    return this->currentSpeedValues[1];
}

/**
 * Callback for the quadcopters own sensor values.
 */
void APIQuadcopter::statusCallback(
    const quadcopter_application::quadcopter_status::ConstPtr &msg) {
    this->linkQuality = msg->link_quality;
    this->stabilizerRollData = msg->stabilizer_roll;
    this->stabilizerPitchData = msg->stabilizer_pitch;
    this->stabilizerYawData = msg->stabilizer_yaw;
    this->batteryStatus = msg->battery_status;
}

/**
 * Callback for the quadcopters position.
 * 
 * @param &msg the received message
 */
void APIQuadcopter::positionCallback(const control_application::quadcopter_position::ConstPtr &msg)
{
    this->currentPositionValues[0] = this->currentPositionValues[0];
    this->currentPositionValues[1] = Vector(msg->x, msg->y, msg->z);
    this->currentPositionTimestamps[0] = this->currentPositionTimestamps[1];
    ros::Time timestamp = msg->header.stamp;
    this->currentPositionTimestamps[1] = timestamp.toSec();
    this->updateSpeed();
}

/**
 * Update the speed value by the current position values.
 */
void APIQuadcopter::updateSpeed()
{
    if (this->currentPositionTimestamps[1] - this->currentPositionTimestamps[0] != 0)
    {
        this->currentSpeedValues[0] = this->currentSpeedValues[1];
        this->currentSpeedTimestamps[0] = this->currentSpeedTimestamps[1];
        //TODO: this might cause a problem when dividing a double by a unsigned double
        double x = (this->currentPositionValues[1].getX() - this->currentPositionValues[0].getX())
        / (this->currentPositionTimestamps[1] - this->currentPositionTimestamps[0]);
        double y = (this->currentPositionValues[1].getY() - this->currentPositionValues[0].getY())
        / (this->currentPositionTimestamps[1] - this->currentPositionTimestamps[0]);
        double z = (this->currentPositionValues[1].getZ() - this->currentPositionValues[0].getZ())
        / (this->currentPositionTimestamps[1] - this->currentPositionTimestamps[0]);
    
        this->currentSpeedValues[1] = Vector(x, y, z);
        this->currentSpeedTimestamps[1] = (int) (this->currentPositionTimestamps[0] + this->currentPositionTimestamps[1]) / 2;
    }
}    

/**
 * Whether this quadcopter is currently tracked.
 * 
 * @return whether this quadcopter i scurrently tracked
 */
bool APIQuadcopter::isTracked()
{
    return !(this->currentPositionValues[1].getX() == -1.0 && this->currentPositionValues[1].getY() == -1.0 && this->currentPositionValues[1].getZ() == -1.0);
}

/**
 * The current roll value from the quadcopter sensors.
 * 
 * @return the roll value
 */
float APIQuadcopter::getStabilizerRollData()
{
    return this->stabilizerRollData;
}

/**
 * The current pitch value from the quadcopter sensors.
 * 
 * @return the pitch value
 */
float APIQuadcopter::getStabilizerPitchData()
{
    return this->stabilizerPitchData;
}

/**
 * The current yaw value from the quadcopter sensors.
 * 
 * @return the yaw value
 */
float APIQuadcopter::getStabilizerYawData()
{
    return this->stabilizerYawData;
}

/**
 * TODO: control_application does not provide any info yet
 */
Vector APIQuadcopter::getCurrentOrientation()
{
    return currentOrientation;
}

/**
 * Get the current position of the quadcopter
 * 
 * @return vector of the current position - lengths are in mm
 */
Vector APIQuadcopter::getCurrentPosition()
{
    return currentPositionValues[1];
}
