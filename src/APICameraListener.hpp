#pragma once

namespace kitrokopter {

class APICameraListener {

	public:
		virtual void updateCameraValues(APICameraUpdateEvent e) = 0;

		virtual ~APICameraListener() { }

};

}
