#include "WorldRect2.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace MyREALM
{


	WorldRect2::WorldRect2() :WorldRect2(0, 0, 0, 0)
	{
	}

	WorldRect2::WorldRect2(const WorldRect2& _rect)
	{
		this->x = _rect.x;
		this->y = _rect.y;
		this->width = _rect.width;
		this->height = _rect.height;
	}

	WorldRect2::WorldRect2(float _x, float _y, float _w, float _h) :
		x(_x), y(_y), width(_w), height(_h)
	{
	}

	cv::Point2f WorldRect2::bottomLeft() const
	{
		return cv::Point2f(x, y);
	}

	cv::Point2f WorldRect2::bottomRight() const
	{
		return cv::Point2f(x + width, y);
	}

	cv::Point2f WorldRect2::topLeft() const
	{
		return cv::Point2f(x, y + height);
	}

	cv::Point2f WorldRect2::topRight() const
	{
		return cv::Point2f(x + width, y + height);
	}

	cv::Size2f WorldRect2::size() const
	{
		return cv::Size2f(width, height);
	}

	bool WorldRect2::empty() const
	{
		return cv::Rect2f(x, y, width, height).empty();
	}

	WorldRect2 WorldRect2::operator&(const WorldRect2& _rect) const
	{
		cv::Rect2f rect1(x, y, width, height);
		cv::Rect2f rect2(_rect.x, _rect.y, _rect.width, _rect.height);
		cv::Rect2f rect3 = rect1 & rect2;
		return WorldRect2(rect3.x, rect3.y, rect3.width, rect3.height);
	}

	WorldRect2& WorldRect2::operator=(const WorldRect2& _rect)
	{
		this->x = _rect.x;
		this->y = _rect.y;
		this->width = _rect.width;
		this->height = _rect.height;
		return *this;
	}

	bool WorldRect2::operator==(const WorldRect2& _rect) const
	{
		return this->x == _rect.x &&
			this->y == _rect.y &&
			this->width == _rect.width &&
			this->height == _rect.height;
	}
}
