#pragma once

namespace kitrokopter {

/**
 * Represents an immutable three dimensional vector.
 */
class Vector {
	private:
		const double x, y, z;

	public:
		Vector(double x, double y, double z) : x(x), y(y), z(z) {}

		double getX() { return x; }
		double getY() { return y; }
		double getZ() { return z; }
};

}
