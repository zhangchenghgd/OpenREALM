#ifndef MYREALM_SCENE_WORLDRECT2_H
#define MYREALM_SCENE_WORLDRECT2_H

#include "myrealm_scene_exports.h"
#include <cassert>
#include <opencv2/core.hpp>

namespace MyREALM
{

	struct MyREALM_Scene_API WorldRect2
	{
		WorldRect2();

		WorldRect2(const WorldRect2& _rect);

		WorldRect2(float _x, float _y, float _w, float _h);

		cv::Point2f bottomLeft() const;

		cv::Point2f bottomRight() const;

		cv::Point2f topLeft() const;

		cv::Point2f topRight() const;

		cv::Size2f size() const;

		bool empty() const;

		WorldRect2 operator &(const WorldRect2& _rect) const;

		WorldRect2& operator = (const WorldRect2& _rect);

		bool operator == (const WorldRect2& _rect) const;

		float x;
		float y;
		float width;
		float height;
	};

}

#endif // !MYREALM_SCENE_WORLDRECT2_H