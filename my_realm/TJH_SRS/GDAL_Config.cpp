/**
 * @file GDAL_Config.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2020-12-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "GDAL_Config.h"
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>
#include <ogr_api.h>
#include <ogr_core.h>
#include <iostream>

namespace TJH
{
    namespace SRS
    {
        void GDAL_Config::setConfigOption(const std::string &option, const std::string &value)
        {
            char opt_name[1024];
            char opt_val[1024];
            sprintf(opt_name,option.c_str());
            sprintf(opt_val,value.c_str());
            CPLSetConfigOption(opt_name, opt_val);
        }

        std::string GDAL_Config::getConfigOption(const std::string &option, const std::string & default_value)
        {
            char opt_name[1024];
            char opt_default[1024];
            sprintf(opt_name,option.c_str());
            sprintf(opt_default,default_value.c_str());
            const char* opt_val = CPLGetConfigOption(opt_name , opt_default);
            std::string val;
            val.assign(opt_val);
            return val;
        }


        std::vector<std::string> GDAL_Config::getPROJSearchPaths()
        {
            std::vector<std::string> paths;
#ifndef USE_GDAL2
            char** proj_paths = OSRGetPROJSearchPaths();
            int num = CSLCount(proj_paths);
            paths.resize(num);
            for(int i=0;i< num; ++i)
            {
                paths[i].assign(proj_paths[i]);
            }
            CSLDestroy(proj_paths);
#endif  /* !USE_GDAL2 */

            return paths;
        }

        void GDAL_Config::setPROJSearchPaths(const std::vector<std::string> &paths)
        {
#ifndef USE_GDAL2
            char **papszStrList = NULL;
            for(std::vector<std::string>::const_iterator iter = paths.cbegin(); 
                iter != paths.cend(); ++iter)
            {
                const char* pth = iter->c_str();
                papszStrList = CSLAddString(papszStrList, pth);
            }
            OSRSetPROJSearchPaths(papszStrList);
            CSLDestroy(papszStrList);
#endif  /* !USE_GDAL2 */
        }


    } // namespace SRS
} // namespace TJH
