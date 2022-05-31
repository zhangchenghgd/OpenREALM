#include "GPSPoseInter.h"



namespace MyREALM
{
	TimeGPSPoseInter::TimeGPSPoseInter() :
		m_expiredMS(60000)
	{
	}

	TimeGPSPoseInter::~TimeGPSPoseInter()
	{
	}

	void TimeGPSPoseInter::putGPS(uint64_t time, const cv::Vec3d& gps)
	{
		if (m_timeGPSs.size() > 0)
		{
			if (time - m_timeGPSs.at(0).first > 60000)
			{
				this->clearExpired(time - 30000);
			}
		}

		m_timeGPSs.push_back(TimeGPS(time, gps));
	}

	void TimeGPSPoseInter::putHeadingAngle(uint64_t time, double heading_angle)
	{
		if (m_timeHeadings.size() > 0)
		{
			if (time - m_timeHeadings.at(0).first > 60000)
			{
				this->clearExpired(time - 30000);
			}
		}

		m_timeHeadings.push_back(TimeHeading(time, heading_angle));
	}

	void TimeGPSPoseInter::clearExpired(uint64_t before_time)
	{
		for (TimeGPSs::iterator ite = m_timeGPSs.begin();
			ite != m_timeGPSs.end(); ite++)
		{
			if (ite->first < before_time)
			{
				m_timeGPSs.erase(ite);
				--ite;
			}
		}

		for (TimeHeadings::iterator ite = m_timeHeadings.begin();
			ite != m_timeHeadings.end(); ite++)
		{
			if (ite->first < before_time)
			{
				m_timeHeadings.erase(ite);
				--ite;
			}
		}
	}

	uint64_t TimeGPSPoseInter::lastGPSTime() const
	{
		if (m_timeGPSs.size() > 0)
		{
			return m_timeGPSs.at(
				m_timeGPSs.size() - 1).first;
		}
		return uint64_t(0);
	}

	uint64_t TimeGPSPoseInter::lastHeadingAngleTime() const
	{
		if (m_timeHeadings.size() > 0)
		{
			return m_timeHeadings.at(
				m_timeHeadings.size() - 1).first;
		}
		return uint64_t(0);
	}
	
	bool TimeGPSPoseInter::interGPS(uint64_t time, cv::Vec3d& gps)
	{
		size_t num_gps = m_timeGPSs.size();
		for (int i = 0; i < num_gps - 1; ++i)
		{
			uint64_t t_before = m_timeGPSs.at(i).first;
			uint64_t t_behind = m_timeGPSs.at(i + 1).first;

			if (time >= t_before && time <= t_behind)
			{
				double bl = (time - t_before) / (t_behind - t_before);

				cv::Vec3d vec_before = m_timeGPSs.at(i).second;

				cv::Vec3d vec_behind = m_timeGPSs.at(i + 1).second;

				cv::Vec3d vec = vec_behind - vec_before;

				gps = vec_before + (vec * bl);

				return true;
			}
		}

		return false;
	}
	
	bool TimeGPSPoseInter::interHeadingAngle(uint64_t time, double& heading_angle)
	{
		size_t num_heads = m_timeHeadings.size();
		for (int i = 0; i < num_heads - 1; ++i)
		{
			uint64_t t_before = m_timeHeadings.at(i).first;
			uint64_t t_behind = m_timeHeadings.at(i + 1).first;

			if (time >= t_before && time <= t_behind)
			{
				double bl = (time - t_before) / (t_behind - t_before);

				double h_before = m_timeHeadings.at(i).second;

				double h_behind = m_timeHeadings.at(i + 1).second;

				double dis = h_behind - h_before;

				heading_angle = h_before + (dis * bl);

				return true;
			}
		}



		return false;
	}

	void TimeGPSPoseInter::clear()
	{
		m_timeGPSs.clear();
		m_timeHeadings.clear();

	}
}
