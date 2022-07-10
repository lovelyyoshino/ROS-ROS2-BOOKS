#ifndef HOGDETECTOR_HPP_
#define HOGDETECTOR_HPP_

#include <opencv2/objdetect/objdetect.hpp>

class HOGDetector : public cv::HOGDescriptor
{
private:
	std::vector<float> defaultModel;
	double searchRoiRatio;

public:
	enum DetectorType
	{
		FULL,
		TORSO,
	};

	HOGDetector();
	HOGDetector(DetectorType);

	std::vector<float> getDefaultModel();
	double getSearchRoiRatio();
};

#endif /* HOGDETECTOR_HPP_ */
