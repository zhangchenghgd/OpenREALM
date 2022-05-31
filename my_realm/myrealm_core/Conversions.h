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

#ifndef PROJECT_CONVERSION_H
#define PROJECT_CONVERSION_H

#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>


#include <realm_core/enums.h>
#include <realm_core/frame.h>
#include <realm_core/utm32.h>
#include <realm_core/structs.h>
#include <realm_core/camera.h>
#include <realm_core/cv_grid_map.h>
#include <realm_core/analysis.h>
#include <realm_core/depthmap.h>
#include "TJH_SRS/Coord.h"
#include "TJH_SRS/Pose.h"
#include "TJH_SRS/UTM.h"

using namespace realm;


namespace MyREALM
{
	bool utm2Wgs84Point(uint8_t zone, char band,
		double easting, double northing, double height,
		double* lon, double* lat, double* alt);

	TJH::SRS::Pose cvPose2SrsPose(const cv::Mat& pose);

	TJH::SRS::Pose utmPose2Wgs84Pose(const TJH::SRS::UTM& utm, const TJH::SRS::Pose& pose);

} 

#endif // !PROJECT_CONVERSION_H
