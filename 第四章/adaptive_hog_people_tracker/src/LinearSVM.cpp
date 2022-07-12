#include "LinearSVM.hpp"

LinearSVM::LinearSVM() //线性SVM划分
{
	params.svm_type = cv::SVM::C_SVC;															   // C_SVC：C-Support Vector Classification
	params.kernel_type = cv::SVM::LINEAR;														   // LINEAR：线性核函数
	params.term_crit = cvTermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-3); //设置终止条件
	params.C = 1;
}

std::vector<float> LinearSVM::trainModel(cv::Mat &data, cv::Mat &labels, std::vector<float> defaultModel)
{
	std::vector<float> model;						   //模型
	train(data, labels, cv::Mat(), cv::Mat(), params); //训练模型
	int sv_count = get_support_vector_count();		   //获取支持向量数量
	const CvSVMDecisionFunc *df = decision_func;	   //获取决策函数
	const double *alphas = df[0].alpha;				   //获取alpha
	double rho = df[0].rho;							   //获取rho
	int var_count = get_var_count();				   //获取变量数量
	model.resize(var_count);						   //设置模型大小

	for (int i = 0; i < sv_count; i++)
	{
		float myalpha = alphas[i];				//获取alpha
		const float *v = get_support_vector(i); //获取支持向量

		for (int j = 0; j < var_count; j++, v++)
		{
			model[j] += (-myalpha) * (*v); //计算模型
		}
	}

	model.push_back(rho);

	double weight = 0.7;

	for (int i = 0; i < model.size(); i++)
	{
		model[i] = (model[i] * weight) + (defaultModel[i] * (1 - weight)); // model和defaultModel计算模型
	}

	return model;
}
