//--------------------------------------------------
// Implementation of class MapMerger
//
// @author: Wild Boar
//
// @date: 2022-06-06
//--------------------------------------------------

#include "MapMerger.h"
using namespace NVL_App;

//--------------------------------------------------
// Merge
//--------------------------------------------------

/**
 * @brief The logic for merging maps
 * @param map1 The first map that we are merging
 * @param map2 The second map that we are merging
 * @param counters The counters for defining the merge weights
 * @return Mat Returns a Mat
 */
Mat MapMerger::Merge(Mat& map1, Mat& map2, Mat& counters)
{
	// Validate the sizes
	assert(map1.rows == map2.rows && map1.cols == map2.cols);

	// Create a result map
	Mat result = Mat_<float>::zeros(map1.size());
	auto result_data = (float *) result.data;

	// Create the handles for extracting data
	auto map1_data = (float *) map1.data;
	auto map2_data = (float *) map2.data;

	// Perform update logic	
	for (auto row = 0; row < map1.rows; row++) 
	{
		for (auto column = 0; column < map1.cols; column++) 
		{
			// Get the current index of the pixel
			auto index = column + row * map1.cols;	

			// Retrieve the Z values
			auto Z_1 = map1_data[index];
			auto Z_2 = map2_data[index];

			// Retrieve the counter value
			auto count = (int)counters.data[index];

			// Validate Depth
			bool valid_1 = IsValid(Z_1); bool valid_2 = IsValid(Z_2);

			// If neither value is valid then just return
			if (!valid_1 && !valid_2) 
			{			
				continue;
			}

			// Handle the various other cases
			else if (valid_1 && !valid_2) { result_data[index] = Z_1; count = 1; }
			else if (!valid_1 && valid_2) { result_data[index] = Z_2; count = 1; }
			else 
			{
				auto combinedZ = (Z_1 * count + Z_2) / (count + 1);
				count = count + 1;
				result_data[index] = combinedZ;
			}
			
			// Update the counter
			counters.data[index] = saturate_cast<uchar>(count);
		}
	}

	// Return the result
	return result;
}
