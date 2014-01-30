#pragma once

#include "APICameraUpdateEvent.hpp"

namespace kitrokopter {

class APICameraListener {

	public:
		virtual void updateCameraValues(APICameraUpdateEvent e) = 0;

		virtual ~APICameraListener() { }

};

}
