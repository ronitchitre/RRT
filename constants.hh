#define _USE_MATH_DEFINES

#include <cmath>

namespace car {
    float max_turn_angle = 10 * (M_PI / 180);
    float car_length = 1;
    float car_velocity = 1;
    float max_turn_rate = car_velocity * tan(max_turn_angle) / car_length;
}