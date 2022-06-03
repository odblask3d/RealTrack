//--------------------------------------------------
// Holds the details associated with the calibration
//
// @author: Wild Boar
//
// @date: 2022-05-31
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

namespace NVL_App
{
	class Calibration
	{
	private:
		Vec2d _focals;
		Point2d _center;
	public:
		Calibration(const Vec2d& focals, const Point2d& center) :
			_focals(focals), _center(center) {}

		inline Mat GetMatrix() 
		{
			Mat camera = Mat_<double>::eye(3,3);

			auto data = (double *) camera.data;
			data[0] = _focals[0]; data[4] = _focals[1];
			data[2] = _center.x; data[5] = _center.y;

			return camera;
		}

		inline Vec2d& GetFocals() { return _focals; }
		inline Point2d& GetCenter() { return _center; }
	};
}
