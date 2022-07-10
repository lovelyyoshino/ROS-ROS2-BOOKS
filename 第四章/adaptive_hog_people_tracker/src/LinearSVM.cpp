#include "LinearSVM.hpp"

LinearSVM::LinearSVM()
{
	params.svm_type = cv::SVM::C_SVC;
	params.kernel_type = cv::SVM::LINEAR;
	params.term_crit = cvTermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-3);
	params.C = 1;
}

std::vector<float> LinearSVM::trainModel(cv::Mat& data, cv::Mat& labels, std::vector<float> defaultModel)
{
	std::vector<float> model;

	train(data, labels, cv::Mat(), cv::Mat(), params);

	int sv_count = get_support_vector_count();

	const CvSVMDecisionFunc* df = decision_func;

	const double* alphas = df[0].alpha;

	double rho = df[0].rho;

	int var_count = get_var_count();

	model.resize(var_count);

	for (int i = 0; i < sv_count; i++)
	{
		float myalpha = alphas[i];
		const float* v = get_support_vector(i);

		for (int j = 0; j < var_count; j++, v++)
		{
			model[j] += (- myalpha) * (* v);
		}
	}

	model.push_back(rho);

	double weight = 0.7;

	for(int i = 0; i < model.size(); i++)
	{
		model[i] = (model[i] * weight) + (defaultModel[i] * (1 - weight));
	}

	return model;
}
