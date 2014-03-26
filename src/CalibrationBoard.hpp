#pragma once

namespace kitrokopter {

/**
 * This class specifies the calibration board used to calibrate the cameras.
 */
class CalibrationBoard {
    public:
	/**
	 * Creates a calibration board with the given dimensions.
	 *
	 * The board's width/height is measured from a single field in.
	 *
	 * @param wdt The board's width.
	 * @param hgt The board's height.
	 * @param boardRectWidth The width of a single field.
	 * @param boardRectHeight The height of a single field.
	 */
	CalibrationBoard(uint32_t wdt, uint32_t hgt, float boardRectWidth, float boardRectHeight) :
	   width(wdt),
	   height(hgt),
	   rectangleWidth(boardRectWidth),
	   rectangleHeight(boardRectHeight)
	{}

	const uint32_t width, height;
	const float rectangleWidth, rectangleHeight;

};

}
