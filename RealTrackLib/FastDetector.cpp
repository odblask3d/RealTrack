//--------------------------------------------------
// Implementation of class FastDetector
//
// @author: Wild Boar
//
// @date: 2022-06-03
//--------------------------------------------------

#include "FastDetector.h"
using namespace NVL_Research;

//--------------------------------------------------
// Extract
//--------------------------------------------------

/**
 * @brief Extract features from the given image
 * @param image The image that we are extracting feature from
 * @param keypoints The list of keypoints that we are extracting from the image
 * @param descriptors Descriptors associated with the application
 */
void FastDetector::Extract(Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors)
{
	Ptr<FeatureDetector> detector = FastFeatureDetector::create();
    vector<KeyPoint> points; detector->detect(image, points);

    auto duplicates = unordered_map<int, KeyPoint>();
    for (const KeyPoint & keypoint : points)
    {
        auto key = GetIndex(keypoint.pt, _blockSize, image.cols);
        auto match = duplicates.find(key);

        if (match == duplicates.end()) duplicates[key] = keypoint;
        else
        {
            if (duplicates[key].response < keypoint.response) duplicates[key] = keypoint;
        }
    }

    for (const pair<int, KeyPoint>& pair : duplicates) keypoints.push_back(pair.second);
}

/**
 * Find the for the point
 * @param point The point that we are getting index of
 * @param blockSize The size of the block
 * @param width The width of the image
 * @return Return a int
 */
int FastDetector::GetIndex(const Point2d& point, int blockSize, int width)
{
    int x = (int)floor(point.x / blockSize); int y = (int)floor(point.y / blockSize);
    return x + y * width;
}

//--------------------------------------------------
// Match
//--------------------------------------------------

/**
 * @brief Match features across images
 * @param kp_1 The list of keypoints from the first image
 * @param kp_2 The list of keypoints from the second image
 * @param descriptor_1 The list of descriptors for the first image
 * @param descriptor_2 The list of descriptors for the second image
 * @param output The list of resultant feature matches for the system
 */
void FastDetector::Match(vector<KeyPoint>& kp_1, vector<KeyPoint>& kp_2, Mat& descriptor_1, Mat& descriptor_2, vector<NVL_Research::FeatureMatch *>& output)
{
	// Validate that the frame is set
	if (_frame == nullptr) throw runtime_error("Right now we are using a setting stereo frame hack - and it was detected that stereo frame was not set.");

	// Prepare the points
    auto points_1 = vector<Point2f>(); for (auto point : kp_1) points_1.push_back(point.pt);
	auto points_2 = vector<Point2f>(); for (auto point : kp_2) points_2.push_back(point.pt);

	// Find the matches
    auto matchPoints = vector<Point2f>(); auto status = vector<uchar>(); auto errors = vector<float>();
    calcOpticalFlowPyrLK(_frame->GetLeft(), _frame->GetRight(), points_1, matchPoints, status, errors, Size(21, 21), 3, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 9000, 1e-8));

	// Perform radius matching with point 2
	FindMatches(points_2, matchPoints, output);

	// Filter based on optical flow error
	FilterOnError(status, errors, output);

	// Filter based on epipolar geometry
	EpipolarFilter(points_1, points_2, output);
}

/**
 * @brief Find matches in the first image 
 * @param pointSet1 The first point set
 * @param pointSet2 The second point set
 * @param matches The list of associated matches
 */
void FastDetector::FindMatches(vector<Point2f>& pointSet1, vector<Point2f>& pointSet2, vector<FeatureMatch *>& matches) 
{
	auto matcher = DescriptorMatcher::create(DescriptorMatcher::MatcherType::FLANNBASED);
	auto matchVector = vector< vector<DMatch> >();

	Mat trainDescriptors = BuildPointDescriptor(pointSet1);
	Mat queryDescriptors = BuildPointDescriptor(pointSet2);
	
	matcher->radiusMatch(queryDescriptors, trainDescriptors, matchVector, 1);

	for (auto i = 0; i < matchVector.size(); i++) 
	{
		if (matchVector[i].size() <= 0) continue;
		auto trainId = matchVector[i][0].queryIdx;
		auto matchId = matchVector[i][0].trainIdx;
		auto score = matchVector[i][0].distance;

		matches.push_back(new FeatureMatch(trainId, matchId, score));
	}
}

/**
 * @brief Build a descriptor consisting of points
 * @param points The points that make up the descriptor
 * @return Mat The resultant descriptor matrix
 */
Mat FastDetector::BuildPointDescriptor(vector<Point2f>& points) 
{
	Mat result = Mat_<float>(points.size(), 2);
	auto data = (float *) result.data;

	for (auto i = 0; i < points.size(); i++) 
	{
		data[i * 2 + 0] = points[i].x;
		data[i * 2 + 1] = points[i].y;
	}

	return result;
}

/**
 * @brief Add the logic to filter on error
 * @param status The status code returned by the feature matcher
 * @param errors The errors returned by the feature matcher
 * @param matches The list of matches
 */
void FastDetector::FilterOnError(vector<uchar>& status, vector<float>& errors, vector<FeatureMatch *>& matches) 
{
	auto counter = 0;
	for (auto i = matches.begin(); i != matches.end(); i++) 
	{
		auto match = *i;
		
		if (status[match->GetFirstId()] == 0 || errors[match->GetFirstId()] > 9) 
		{
			i = matches.erase(i);
			delete match;	
			if (i == matches.end()) break;
		}
	}
}

/**
 * Filter matches
 * @param pointSet1 The first set of points that we are working with
 * @param pointSet2 The second set of points that we are working with
 * @param output The output list of matches
 */
void FastDetector::EpipolarFilter(vector<Point2f>& pointSet1, vector<Point2f>& pointSet2, vector<NVL_Research::FeatureMatch *>& output)
{
   	// Extract the matching points 
   	vector<Point2f> points_1; vector<Point2f> points_2;
	for (auto match : output) 
	{ 
		points_1.push_back(pointSet1[match->GetFirstId()]); 
		points_2.push_back(pointSet2[match->GetSecondId()]); 
	}

	// Find the fundamental matrix and extract the outliers
    auto mask = vector<uchar>();
    auto F = findFundamentalMat(points_1, points_2, FM_LMEDS, 1.0, 0.8, mask);

	auto counter = 0;
	for (auto i = output.begin(); i != output.end(); i++) 
	{
		auto match = *i;
		
		auto firstId = match->GetFirstId();

		if (mask[counter++] == 0) 
		{
			i = output.erase(i);
			delete match;
			if (i == output.end()) break;	
		}
	}
}

//--------------------------------------------------
// SetFrame
//--------------------------------------------------

/**
 * @brief Defines the logic to set the associated stereo frame
 * @param image1 The first image in the collection
 * @param image2 The second image in the collection
 */
void FastDetector::SetFrame(Mat& image1, Mat& image2) 
{
	if (_frame != nullptr) delete _frame;
	_frame = new NVLib::StereoFrame(image1, image2);
}
