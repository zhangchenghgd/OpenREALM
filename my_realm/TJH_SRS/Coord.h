/**
 * @file Coord.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 坐标点的定义，包括二维点Coord2和三维点Coord3
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_COORD_H_
#define TJH_SRS_COORD_H_

#include "SrsExport.h"
#include <iostream>
#include <cstring>

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 未定义的坐标值
         */
        const double UndefCoordVal = DBL_MAX;

        /**
         * @brief 二维坐标
         * @details
         *        空间直角坐标系：(X, Y)；\n
         *        地理坐标系：(Longitude, Latitude) 或者 (L, B)；\n
         *        投影坐标系：(East, North)；\n
         */
        struct TJH_SRS_EXPORT Coord2
        {
            /**
             * @brief 数据内容,存储XY坐标值
             */
            double data[2];

            /**
             * @brief 横坐标，X/East(东)/Longitude(经度，正：东经，负：西经)
             */
            const double &x() const;

            /**
             * @brief 纵坐标，Y/North(北)/Latitude(纬度，正：北纬，负：南纬)
             */
            const double &y() const;

            /**
             * @brief 横坐标，X/East(东)/Longitude(经度，正：东经，负：西经)
             */
            double &x();

            /**
             * @brief 纵坐标，Y/North(北)/Latitude(纬度，正：北纬，负：南纬)
             */
            double &y();

            /**
             * @brief 构造 Coord 2 对象
             */
            Coord2();

            /**
             * @brief 构造 Coord 2 对象
             * @param _x 
             * @param _y 
             */
            Coord2(double _x, double _y);

            /**
             * @brief 设置x和y值
             * @param _x 
             * @param _y 
             */
            void set(double _x, double _y);

            /**
             * @brief 值是否都为0
             * @return true 
             * @return false 
             */
            bool isZero() const;

            /**
             * @brief 坐标值是否为空，空值： UndefCoordVal
             * @return true 
             * @return false 
             */
            bool empty() const;

            /**
             * @brief 重置
             */
            void reset();

            /**
             * @brief 点坐标向量的模
             * @return double 
             */
            double norm() const;

            /**
             * @brief 运算符 = 重载
             * @param other 
             * @return Coord2& 
             */
            Coord2 &operator=(const Coord2 &other);

            /**
             * @brief 运算符 + 重载
             * @param coord1 
             * @return Coord2 
             */
            Coord2 operator+(const Coord2 &coord1) const;

            Coord2 operator-(const Coord2 &coord1) const;

            bool operator==(const Coord2 &coord1) const;

            bool operator!=(const Coord2 &coord1) const;

            template <typename T>
            friend Coord2 operator*(const T &s, const Coord2 &coord1);

            template <typename T>
            inline Coord2 operator*(const T &s) const
            {
                return Coord2(data[0] * s, data[1] * s);
            }

            template <typename T>
            inline Coord2 operator/(const T &s) const
            {
                return Coord2(data[0] / s, data[1] / s);
            }

            friend std::istream &operator>>(std::istream&, Coord2 &coord);

            friend std::ostream &operator<<(std::ostream&, const Coord2 &coord);

            /**
             * @brief 坐标值为0
             */
            inline static Coord2 zero() { return Coord2(0.0, 0.0); } 
        };

        /**
         * @brief 三维坐标
         * @details
         *        空间直角坐标系：(X, Y, Z)；\n
         *        地理坐标系：(Longitude, Latitude, Altitude) 或者 (L, B, H)；\n
         *        投影坐标系：(East, North, Z)；\n
         */
        struct TJH_SRS_EXPORT Coord3
        {
            /**
             * @brief 数据内容,存储XYZ坐标值
             */
            double data[3];

            /**
             * @brief 横坐标，X/East(东)/Longitude(经度，正：东经，负：西经)
             */
            const double &x() const;

            /**
             * @brief 纵坐标，Y/North(北)/Latitude(纬度，正：北纬，负：南纬)
             * 
             */
            const double &y() const;

            /**
             * @brief 纵坐标，Z/Height(高)/Altitude(海拔)
             */
            const double &z() const;

            /**
             * @brief 横坐标，X/East(东)/Longitude(经度，正：东经，负：西经)
             * 
             */
            double &x();

            /**
             * @brief 纵坐标，Y/North(北)/Latitude(纬度，正：北纬，负：南纬)
             */
            double &y();

            /**
             * @brief 纵坐标，Z/Height(高)/Altitude(海拔)
             */
            double &z();

            /**
             * @brief 构造 Coord 3 对象
             */
            Coord3();

            /**
             * @brief 构造 Coord 3 对象
             * @param _x 
             * @param _y 
             * @param _z 
             */
            Coord3(double _x, double _y, double _z);

            /**
             * @brief 构造 Coord 3 对象
             * @param _coord2 
             * @param _z 
             */
            Coord3(const Coord2 &_coord2, double _z = 0.0);

            /**
             * @brief 设置xyz值
             * @param _x 
             * @param _y 
             * @param _z 
             */
            void set(double _x, double _y, double _z);

            /**
             * @brief 值是否都为0
             * @return true 
             * @return false 
             */
            bool isZero() const;

            /**
             * @brief 坐标值是否为空，空值： UndefCoordVal
             * @return true 
             * @return false 
             */
            bool empty() const;

            /**
             * @brief 重置
             * 
             */
            void reset();

            /**
             * @brief 点坐标向量的模
             * @return double 
             */
            double norm() const;

            Coord3 &operator=(const Coord3 &other);

            Coord3 operator+(const Coord3 &coord1) const;

            Coord3 operator-(const Coord3 &coord1) const;

            bool operator==(const Coord3 &coord1) const;

            bool operator!=(const Coord3 &coord1) const;


            template <typename T>
            friend Coord3 operator*(const T &s, const Coord3 &coord1);

            template <typename T>
            inline Coord3 operator*(const T &s) const
            {
                return Coord3(data[0] * s, data[1] * s, data[2] * s);
            }

            template <typename T>
            inline Coord3 operator/(const T &s) const
            {
                return Coord3(data[0] / s, data[1] / s, data[2] / s);
            }

            friend std::istream &operator>>(std::istream&, Coord3 &coord);

            friend std::ostream &operator<<(std::ostream&, const Coord3 &coord);

            /**
             * @brief 坐标值为0
             */
            inline static Coord3 zero() { return Coord3(0.0, 0.0, 0.0); } 

        };

        /**
         * @brief 经纬度海拔类型
         */
        typedef Coord3 LonLatAlt;

        /**
         * @brief 经纬度
         */
        typedef Coord2 LonLat;

        template <typename T>
        Coord2 operator*(const T &s, const Coord2 &coord1)
        {
            return Coord2(coord1.x() * s, coord1.y() * s);
        }

        inline std::istream &operator>>(std::istream& in, Coord2 &coord)
        {
            in >> coord.data[0] >> coord.data[1];
            return in;
        }

        inline std::ostream &operator<<(std::ostream& out, const Coord2 &coord)
        {
            out << coord.data[0] <<"  "<< coord.data[1];
            return out;
        }

        template <typename T>
        inline Coord3 operator*(const T &s, const Coord3 &coord1)
        {
            return Coord3(coord1.x() * s, coord1.y() * s, coord1.z() * s);
        }

        inline std::istream &operator>>(std::istream& in, Coord3 &coord)
        {
            in >> coord.data[0] >> coord.data[1] >> coord.data[2];
            return in;
        }

        inline std::ostream &operator<<(std::ostream& out, const Coord3 &coord)
        {
            out << coord.data[0] <<"  "<< coord.data[1] <<"  "<< coord.data[2];
            return out;
        }

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_COORD_H_
