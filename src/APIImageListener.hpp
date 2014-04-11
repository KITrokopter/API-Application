#pragma once

#include <opencv2/core/core.hpp>

namespace kitrokopter {
class APIImageListener {
public:

	virtual void imageReceived(cv::Mat)  = 0;

	// virtual destructor for interface
	virtual ~APIImageListener()
	{
	}
};
}
