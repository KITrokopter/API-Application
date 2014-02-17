#pragma once

namespace kitrokopter {

/**
 * Represents an immutable three dimensional vector.
 */
class Vector {
	private:
		const double x, y, z;

	public:
		/**
		 * Constructs a vector with the given coordinates.
		 *
		 * @param x The first coordinate.
		 * @param y The second coordinate.
		 * @param z The third coordinate.
		 */
		Vector(double x, double y, double z) : x(x), y(y), z(z) {}

		/**
		 * @return The first coordinate.
		 */
		double getX() { return x; }

		/**
		 * @return The second coordinate.
		 */
		double getY() { return y; }

		/**
		 * @return The third coordinate.
		 */
		double getZ() { return z; }
};

}
