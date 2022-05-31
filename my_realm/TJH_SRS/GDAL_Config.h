/**
 * @file GDAL_Config.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 配置GDAL和Proj选项
 * @version 0.1
 * @date 2020-12-18
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */

#ifndef TJH_SRS_GDAL_CONFIG_H_
#define TJH_SRS_GDAL_CONFIG_H_

#include "SrsExport.h"
#include <string>
#include <vector>

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 用于本项目的GDAL和PROJ选项设置
         */
        class TJH_SRS_EXPORT GDAL_Config
        {
        public:
            /**
             * @brief 设置GDAL选项
             * @param[in] option 
             * @param[in] value 
             */
            static void setConfigOption(const std::string &option, const std::string &value);

            /**
             * @brief 设置GDAL选项
             * @param option 
             * @return std::string 
             */
            static std::string getConfigOption(const std::string &option, const std::string & default_value = "");


            /**
             * @brief 获取PROJ路径，若使用 GDAL2 函数无效
             * @return std::vector<std::string> 
             */
            static std::vector<std::string> getPROJSearchPaths();

            /**
             * @brief 设置PROJ路径，若使用 GDAL2 函数无效
             * @param paths 
             */
            static void setPROJSearchPaths(const std::vector<std::string> &paths);

        };

    } // namespace SRS
} // namespace TJH

#endif //  !TJH_SRS_GDAL_CONFIG_H_