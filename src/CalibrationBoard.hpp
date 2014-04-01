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
	 * @param hnum The number of horizontal fields.
	 * @param vnum The number of vertical fields.
	 * @param boardRectWidth The width of a single field.
	 * @param boardRectHeight The height of a single field.
	 */
	CalibrationBoard(uint32_t hnum, uint32_t vnum, float boardRectWidth, float boardRectHeight) :
	   horizontalNumber(hnum),
	   verticalNumber(vnum),
	   rectangleWidth(boardRectWidth),
	   rectangleHeight(boardRectHeight)
	{}

	const uint32_t horizontalNumber, verticalNumber;
	const float rectangleWidth, rectangleHeight;

};

}
