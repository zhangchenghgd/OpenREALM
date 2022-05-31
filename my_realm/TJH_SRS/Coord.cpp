/**
 * @file Coord.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "Coord.h"
#include <cmath>

namespace TJH
{
    namespace SRS
    {
        Coord2::Coord2()
        {
            data[0] = UndefCoordVal;
            data[1] = UndefCoordVal;
        }

        Coord2::Coord2(double _x, double _y)
        {
            data[0] = _x;
            data[1] = _y;
        }

        const double &Coord2::x() const { return data[0]; }

        const double &Coord2::y() const { return data[1]; }

        double &Coord2::x() { return data[0]; }

        double &Coord2::y() { return data[1]; }

        void Coord2::set(double _x, double _y)
        {
            data[0] = _x;
            data[1] = _y;
        }

        bool Coord2::isZero() const
        {
            return data[0] == 0 && data[1] == 0;
        }

        bool Coord2::empty() const
        {
            return data[0] == UndefCoordVal &&
                   data[1] == UndefCoordVal;
        }

        void Coord2::reset()
        {
            data[0] = data[1] = UndefCoordVal;
        }

        Coord2 &Coord2::operator=(const Coord2 &other)
        {
            memcpy(data, other.data, 2 * sizeof(double));
            return *this;
        }

        Coord2 Coord2::operator+(const Coord2 &coord) const
        {
            return Coord2(data[0] + coord.data[0],
                          data[1] + coord.data[1]);
        }

        Coord2 Coord2::operator-(const Coord2 &coord) const
        {
            return Coord2(data[0] - coord.data[0],
                          data[1] - coord.data[1]);
        }

        bool Coord2::operator==(const Coord2 &coord) const
        {
            return (data[0] == coord.data[0] &&
                    data[1] == coord.data[1]) == true;
        }

        bool Coord2::operator!=(const Coord2 &coord) const
        {
            return (data[0] == coord.data[0] &&
                    data[1] == coord.data[1]) == false;
        }

        double Coord2::norm() const
        {
            return sqrt(data[0] * data[0] +
                        data[1] * data[1]);
        }

        Coord3::Coord3()
        {
            data[0] = UndefCoordVal;
            data[1] = UndefCoordVal;
            data[2] = UndefCoordVal;
        }

        Coord3::Coord3(double _x, double _y, double _z)
        {
            data[0] = _x;
            data[1] = _y;
            data[2] = _z;
        }

        Coord3::Coord3(const Coord2 &_coord2, double _z)
        {
            data[0] = _coord2.x();
            data[1] = _coord2.y();
            data[2] = _z;
        }

        const double &Coord3::x() const { return data[0]; }

        const double &Coord3::y() const { return data[1]; }

        const double &Coord3::z() const { return data[2]; }

        double &Coord3::x() { return data[0]; }

        double &Coord3::y() { return data[1]; }

        double &Coord3::z() { return data[2]; }

        void Coord3::set(double _x, double _y, double _z)
        {
            data[0] = _x;
            data[1] = _y;
            data[2] = _z;
        }

        bool Coord3::isZero() const
        {
            return data[0] == 0 &&
                   data[1] == 0 &&
                   data[2] == 0;
        }

        bool Coord3::empty() const
        {
            return data[0] == UndefCoordVal &&
                   data[1] == UndefCoordVal &&
                   data[2] == UndefCoordVal;
        }

        void Coord3::reset()
        {
            data[0] = data[1] = data[2] = UndefCoordVal;
        }

        Coord3 &Coord3::operator=(const Coord3 &other)
        {
            memcpy(data, other.data, 3 * sizeof(double));
            return *this;
        }

        Coord3 Coord3::operator+(const Coord3 &coord) const
        {
            return Coord3(data[0] + coord.data[0],
                          data[1] + coord.data[1],
                          data[2] + coord.data[2]);
        }

        Coord3 Coord3::operator-(const Coord3 &coord) const
        {
            return Coord3(data[0] - coord.data[0],
                          data[1] - coord.data[1],
                          data[2] - coord.data[2]);
        }

        bool Coord3::operator==(const Coord3 &coord1) const
        {
            return (data[0] == coord1.data[0] &&
                    data[1] == coord1.data[1] &&
                    data[2] == coord1.data[2]) == true;
        }

        bool Coord3::operator!=(const Coord3 &coord1) const
        {
            return (data[0] == coord1.data[0] &&
                    data[1] == coord1.data[1] &&
                    data[2] == coord1.data[2]) == false;
        }

        double Coord3::norm() const
        {
            return sqrt(data[0] * data[0] +
                        data[1] * data[1] +
                        data[2] * data[2]);
        }

    } // namespace SRS

} // namespace TJH
