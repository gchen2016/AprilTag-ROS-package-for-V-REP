#ifndef PATH_UTIL_H
#define PATH_UTIL_H

#include <cmath>
#include <stdlib.h>

namespace path_util {
	enum Direction {
		UP,
		LEFT,
		DOWN,
		RIGHT
	};

	int ThetaDistance(int t1, int t2) {
		return (abs(t1 - t2) < 3) ? abs(t1 - t2) : (abs(t1 -t2) - 2);
	}

	int ManDistance(int x1, int y1, int t1, int x2, int y2, int t2) {
		return (abs(x1 - x2) + abs(y1 - y2) + ThetaDistance(t1, t2));
	}

	int getYawEnum(float yaw_f) {
		return (int)round((yaw_f + 3.1415926) / 1.5707963);
	}
	float getYawFloat(int yaw_i) {
		return (float)yaw_i * 1.5707963 - 3.1415926;
	}
}

#endif