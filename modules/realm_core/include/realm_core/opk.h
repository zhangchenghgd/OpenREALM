#ifndef PROJECT_OPK_H
#define PROJECT_OPK_H


#include <opencv2/core.hpp>

namespace realm
{


	/**
	 * @brief 转换：旋转矩阵3x3 => Omega-Phi-Kappa角元素
	 * @param[in] R         旋转矩阵3x3，同ContextCapture中的旋转矩阵表示一致
	 * @param[out] omega     omega，单位：角度
	 * @param[out] phi       phi，单位：角度
	 * @param[out] kappa     kappa，单位：角度
	 */
	void RotationMatrix_To_EulerAngleOPK(
		const cv::Mat& R,
		double& omega,
		double& phi,
		double& kappa);

	/**
	 * @brief 转换：Omega-Phi-Kappa角元素 => 旋转矩阵3x3
	 * @param[in] _omega    omega，单位：角度
	 * @param[in] _phi      phi，单位：角度
	 * @param[in] _kappa    kappa，单位：角度
	 * @param[out] R        旋转矩阵3x3，同ContextCapture中的旋转矩阵表示一致
	 */
	void EulerAngleOPK_To_RotationMatrix(
		double _omega,
		double _phi,
		double _kappa,
		cv::Mat& R);


}

#endif // !PROJECT_OPK_H
