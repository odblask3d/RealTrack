//--------------------------------------------------
// Defines the matching of corresponding feature points
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

namespace NVL_Research
{
	class FeatureMatch
	{
	private:
		int _firstId;
		int _secondId;
		double _matchScore;
	public:
		FeatureMatch(int firstId, int secondId, double matchScore) :
			_firstId(firstId), _secondId(secondId), _matchScore(matchScore) {}

		inline int& GetFirstId() { return _firstId; }
		inline int& GetSecondId() { return _secondId; }
		inline double& GetMatchScore() { return _matchScore; }
	};
}