#ifndef LINEARSVM_HPP_
#define LINEARSVM_HPP_

#include <opencv2/ml/ml.hpp>

class LinearSVM : public cv::SVM
{
	private:
		cv::SVMParams params;

	public:
		LinearSVM();
		std::vector<float> trainModel(cv::Mat& data, cv::Mat& labels, std::vector<float> defaultModel);
};

#endif /* LINEARSVM_HPP_ */
