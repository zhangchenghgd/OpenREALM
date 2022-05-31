/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Conversions.h"


namespace MyREALM
{
	bool utm2Wgs84Point(uint8_t zone, char band,
		double easting, double northing, double height, 
		double* lon, double* lat, double* alt)
	{
		bool b_south = band >= 'C' && band <= 'M';
		bool b_north = band >= 'N' && band <= 'X';

		if (!b_south && !b_north) { return false; }
		
		TJH::SRS::UTM utm(zone, b_north);
		TJH::SRS::LonLatAlt lla = utm.toLonLatAlt(
			TJH::SRS::Coord3(easting, northing, height));

		*lon = lla.x();
		*lat = lla.y();
		*alt = lla.z();

		return true;
	}

	TJH::SRS::Pose cvPose2SrsPose(const cv::Mat& pose)
	{

		TJH::SRS::Pose srs_pose;

		srs_pose.setCenter(TJH::SRS::Coord3(
			pose.at<double>(0, 3),
			pose.at<double>(1, 3),
			pose.at<double>(2, 3)));

		double R[9] = {
			pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
			pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2),
			pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2) };

		srs_pose.setRotation(&R[0]);
		
		return srs_pose;
	}

	TJH::SRS::Pose utmPose2Wgs84Pose(const TJH::SRS::UTM& utm, const TJH::SRS::Pose& pose)
	{
		utm.epsg();

		return TJH::SRS::Pose();
	}



} // namespace realm
