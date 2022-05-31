#include <realm_core/opk.h>
#include <math.h>


namespace realm
{

	void RotationMatrix_To_EulerAngleOPK(const cv::Mat& R, double& omega, double& phi, double& kappa)
	{
		phi = asin(R.at<double>(2, 0)) * 180.0 / M_PI;
		omega = atan2(-R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / M_PI;
		kappa = atan2(-R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / M_PI;
	}

	void EulerAngleOPK_To_RotationMatrix(double _omega, double _phi, double _kappa, cv::Mat& R)
	{
		double omega = _omega * M_PI / 180.0;
		double phi = _phi * M_PI / 180.0;
		double kappa = _kappa * M_PI / 180.0;

		R.at<double>(0, 0) = cos(phi) * cos(kappa);
		R.at<double>(0, 1) = cos(omega) * sin(kappa) + sin(omega) * sin(phi) * cos(kappa);
		R.at<double>(0, 2) = sin(omega) * sin(kappa) - cos(omega) * sin(phi) * cos(kappa);

		R.at<double>(1, 0) = -cos(phi) * sin(kappa);
		R.at<double>(1, 1) = cos(omega) * cos(kappa) - sin(omega) * sin(phi) * sin(kappa);
		R.at<double>(1, 2) = sin(omega) * cos(kappa) + cos(omega) * sin(phi) * sin(kappa);

		R.at<double>(2, 0) = sin(phi);
		R.at<double>(2, 1) = -sin(omega) * cos(phi);
		R.at<double>(2, 2) = cos(omega) * cos(phi);
	}

}

