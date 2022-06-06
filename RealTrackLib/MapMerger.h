//--------------------------------------------------
// Contains the logic to merging multiple depth maps
//
// @author: Wild Boar
//
// @date: 2022-06-06
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

namespace NVL_App
{
	class MapMerger
	{
	public:
		static Mat Merge(Mat& map1, Mat& map2, Mat& counters);
	private:

		inline static bool IsValid(float Z) 
		{
			return Z > 300;
		}


	};
}