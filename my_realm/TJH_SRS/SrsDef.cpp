#include "SrsDef.h"
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>
#include <sstream>
#include <iomanip>
#include <ios>
#include <map>
#include <assert.h>

namespace TJH
{
    namespace SRS
    {

        std::string LinearUnitString(LinearUnits unit)
        {
            static std::map<LinearUnits, std::string> unit_map{
                {LinearUnits::arbitrary, ""},
                {LinearUnits::meter, SRS_UL_METER},
                {LinearUnits::foot, SRS_UL_FOOT},
                {LinearUnits::us_foot, SRS_UL_US_FOOT}};

            return unit_map[unit];
        }

        double LinearUnitInMeters(LinearUnits unit)
        {
            static std::map<LinearUnits, double> unit_map{
                {arbitrary, 0.0},
                {meter, 1.0 },
                {foot, atof(SRS_UL_FOOT_CONV) },
                {us_foot, atof(SRS_UL_US_FOOT_CONV) }};
            return unit_map[unit];
        }

        SrsDef::SrsDef()
        {
        }

        SrsDefLocalCartesian::SrsDefLocalCartesian(LinearUnits u) : SrsDef(), Units(u)
        {
        }

        std::string SrsDefLocalCartesian::toString() const
        {
            static std::map<LinearUnits, std::string> unit_map{
                {arbitrary, "0"},
                {meter, "Meter"},
                {foot, "Foot"},
                {us_foot, "Foot_US"}};

            std::stringstream def_str;
            def_str << "Local:unit=" << unit_map[Units];

            return def_str.str();
        }

        SrsDefENU::SrsDefENU(double lat, double lon) : m_origin(lon, lat)
        {
        }

        SrsDefENU::SrsDefENU(const Coord2 &_origin) : m_origin(_origin)
        {
        }

        std::string SrsDefENU::toString() const
        {
            std::stringstream ss;
			// 保留小数点精度16位
            ss << "ENU:" << std::setiosflags(std::ios::fixed) << std::setprecision(16) 
				<< m_origin.y() << "," << m_origin.x();
            return ss.str();
        }

        SrsDefEPSG::SrsDefEPSG(int epsg) : SrsDef(), m_epsg(epsg),
                                           m_vertical_epsg(0)
        {
        }

        SrsDefEPSG::SrsDefEPSG(int epsg, int vertical_epsg) : SrsDef(),
                                                              m_epsg(epsg),
                                                              m_vertical_epsg(vertical_epsg)
        {
        }

        std::string SrsDefEPSG::toString() const
        {
            std::stringstream ss;
            ss << "EPSG:" << m_epsg;
            if (m_vertical_epsg > 0)
            {
                ss << "+" << m_vertical_epsg;
            }
            return ss.str();
        }

        SrsDefWKT::SrsDefWKT(const std::string &wkt) : SrsDef(),
                                                       m_wkt(wkt)
        {
        }

        std::string SrsDefWKT::toString() const
        {
            return m_wkt;
        }

        SrsDefProj4::SrsDefProj4(const std::string &proj4) : SrsDef(),
                                                             m_proj4(proj4)
        {
        }

        std::string SrsDefProj4::toString() const
        {
            return m_proj4;
        }

    } // namespace SRS
} // namespace TJH