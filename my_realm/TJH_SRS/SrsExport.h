/**
 * @file SrsExport.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief SrsExport导出相关定义
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_EXPORT_H_
#define TJH_SRS_EXPORT_H_


# if WIN32
    # if defined(TJH_SRS_LIB)
    #  define TJH_SRS_EXPORT __declspec(dllexport)
    # else
    #  define TJH_SRS_EXPORT  __declspec(dllimport)
    # endif
# else
    #define TJH_SRS_EXPORT
#endif


#endif  // TJH_SRS_EXPORT_H_ 
