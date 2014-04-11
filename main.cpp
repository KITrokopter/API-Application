#include "API.hpp"

int main(int argc, char **argv)
{
	new kitrokopter::API(argc, argv, true);
	ros::shutdown();

	// Wait for ros to shutdown.
	while (ros::ok()) {
		usleep(10000);
	}

	std::cout << "API Application successfully terminated" << std::endl;
}

