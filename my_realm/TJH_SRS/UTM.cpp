#include "UTM.h"
#include "SrsTransform.h"
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>
#include <ogr_api.h>
#include <ogr_core.h>
#include <assert.h>

namespace TJH
{
    namespace SRS
    {
        class UTMPrivate
        {
            friend class UTM;

        public:
            SrsTransform *WGS84_TO_UTM;
            SrsTransform *UTM_TO_WGS84;
            int UTM_EPSG;
            std::string UTM_NAME;

            UTMPrivate()
            {
                WGS84_TO_UTM = new SrsTransform;
                UTM_TO_WGS84 = new SrsTransform;
                UTM_EPSG = 0;
                UTM_NAME = "";
            }

            ~UTMPrivate()
            {
                delete WGS84_TO_UTM;
                delete UTM_TO_WGS84;
            }

            void setUTM(int zone, bool north)
            {
                std::shared_ptr<SrsDef> wgs84_srs = std::make_shared<SrsDefEPSG>(4326);

				char name_buf[512];
				sprintf(name_buf, "WGS 84 / UTM zone %d%s", zone, (north ? "N" : "S"));
				UTM_NAME.assign(name_buf);

                OGRSpatialReference *UTM_SR = new OGRSpatialReference;
				UTM_SR->SetWellKnownGeogCS("WGS84");
                OGRErr ogr_err = UTM_SR->SetUTM(zone, north ? 1 : 0);
                assert(ogr_err == OGRERR_NONE);
                ogr_err = UTM_SR->AutoIdentifyEPSG();
				char* utm_wkt = nullptr;
				UTM_SR->exportToPrettyWkt(&utm_wkt);

                assert(ogr_err == OGRERR_NONE);
                const char *auth_code = UTM_SR->GetAuthorityCode(NULL);
				assert(auth_code != NULL);
                UTM_EPSG = atoi(auth_code);
                UTM_SR->Clear();
                ogr_err = UTM_SR->importFromEPSG(UTM_EPSG);
                assert(ogr_err == OGRERR_NONE);

                //UTM_NAME = UTM_SR->GetName();

                OGRSpatialReference::DestroySpatialReference(UTM_SR);
				UTM_SR = NULL;

                std::shared_ptr<SrsDef> utm_srs = std::make_shared<SrsDefEPSG>(UTM_EPSG);

                WGS84_TO_UTM->setTransformSRS(wgs84_srs, utm_srs);
                UTM_TO_WGS84->setTransformSRS(utm_srs, wgs84_srs);

				utm_srs = NULL;
				wgs84_srs = NULL;
            }
        };

        UTM::UTM(int p_zone, bool p_north) : m_zone(p_zone), m_north(p_north)
        {
            m_UTMPrivate = new UTMPrivate;
            m_UTMPrivate->setUTM(p_zone, p_north);
        }

        UTM::~UTM()
        {
            delete m_UTMPrivate;
        }

        int UTM::zone() const
        {
            return m_zone;
        }

        bool UTM::north() const
        {
            return m_north;
        }

        int UTM::epsg() const
        {
            return m_UTMPrivate->UTM_EPSG;
        }

        std::string UTM::name() const
        {
            return m_UTMPrivate->UTM_NAME;
        }

        Coord3 UTM::fromLonLatAlt(const LonLatAlt &lla_coord) const
        {
            return m_UTMPrivate->WGS84_TO_UTM->operator()(lla_coord);
        }

        LonLatAlt UTM::toLonLatAlt(const Coord3 &utm_coord) const
        {
            return m_UTMPrivate->UTM_TO_WGS84->operator()(utm_coord);
        }

        UTM UTM::locateUTM(const LonLat &lon_lat)
        {
            // 断言：经度范围[-180.0, 180.0]，纬度范围[-90.0, 90.0]
            assert(lon_lat.x() >= -180.0 && lon_lat.x() <= 180.0);
            assert(lon_lat.y() >= -90.0 && lon_lat.y() <= 90.0);

            int utm_zone = (int)(lon_lat.x()) / 6 + 31;
            bool north = lon_lat.y() >= 0;
            return UTM(utm_zone, north);
        }

    }
}