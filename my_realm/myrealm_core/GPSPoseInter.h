#ifndef MY_REALM_GPSPOSE_INTER_H
#define MY_REALM_GPSPOSE_INTER_H

#include "opencv2/opencv.hpp"
#include <cstdint>
#include <utility>
#include <vector>

namespace MyREALM
{

	typedef std::pair<uint64_t, cv::Vec3d> TimeGPS;
	typedef std::pair<uint64_t, double> TimeHeading;

	typedef std::vector<TimeGPS> TimeGPSs;
	typedef std::vector<TimeHeading> TimeHeadings;

	// GPS和Heading角度插值
	class TimeGPSPoseInter
	{
	public:
		TimeGPSPoseInter();
		~TimeGPSPoseInter();

		// 放入GPS和时间
		void putGPS(uint64_t time, const cv::Vec3d& gps);

		// 放入GPS和时间
		void putHeadingAngle(uint64_t time, double heading_angle);

		void clearExpired(uint64_t before_time);

		uint64_t lastGPSTime() const;

		uint64_t lastHeadingAngleTime() const;

		bool interGPS(uint64_t time, cv::Vec3d& gps);

		bool interHeadingAngle(uint64_t time, double& heading_angle);

		void clear();

	private:
		TimeGPSs m_timeGPSs;
		TimeHeadings m_timeHeadings;
		uint64_t m_expiredMS;
	};

}


#endif // !MY_REALM_GPSPOSE_INTER_H
