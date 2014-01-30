#pragma once

#include <string>

namespace kitrokopter {

class APIMessageListener {

	public:
		virtual void apiErrorMessage(std::string error)  = 0;
		virtual void apiWarningMessage(std::string warning)  = 0;
		virtual void apiInfoMessage(std::string info)  = 0;

		// virtual destructor for interface 
		virtual ~APIMessageListener() { }

};

}
