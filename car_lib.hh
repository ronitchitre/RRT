#include "constants.hh"
#include <eigen3/Eigen/Dense>
#include <cmath>

float ang_between(Eigen::Vector2f v1, Eigen::Vector2f v2) {
    float cross = v1(0) * v2(1) - v1(1) * v2(0);
    if (abs(cross) < 0.01f) {
        return 0.0;
    }
    return asin(cross / (v1.norm() * v2.norm()));

}

Eigen::Vector2f normalize(Eigen::Vector2f v) {
    if (v.norm() != 0) {
        v = v / v.norm();
    }
    return v;
}

Eigen::Vector2f rotate(Eigen::Vector2f v, float theta, float l) {
    Eigen::Matrix<float, 2, 2> rot_mat;
    rot_mat << cos(theta), -1 * sin(theta),
                sin(theta), cos(theta);
    v = rot_mat * v;
    v = (v / v.norm()) * l;
    return v;
}

class Car{
    private:
        Eigen::Vector2f x;
        Eigen::Vector2f v;
        float theta;
    
    public:
    Car(Eigen::Vector2f x, Eigen::Vector2f v, float theta) {
        this->x = x;
        this->v = v;
        this->theta = theta;
    }
    void propogate(Node &des_node, float &distance) {
        distance = -1;
        Eigen::Vector2d des_direction;
        float alpha;
        float dist_between_nodes;
        float omega;
        float time_taken;
        Eigen::Vector2f  new_v;
        float new_theta;
        Eigen::Vector2f new_x;
        des_direction = des_node.x - this->x;
        alpha = ang_between(this->v, des_direction);
        dist_between_nodes = des_direction.norm();
        omega = 2 * car::car_velocity * sin(alpha) / dist_between_nodes;
        if (abs(omega) > car::max_turn_angle) {
            return;
        }
        if (abs(omega) < 0.01f) {
            time_taken = 2 * alpha / omega;
        }
        else {
            time_taken = dist_between_nodes / car::car_velocity;
        }
        distance = time_taken * car::car_velocity;
        new_v = rotate(this->v, 2 * alpha, car::car_velocity);
        new_theta = this->theta + omega * time_taken;
        new_x = des_node.x;
        des_node = Node(new_x, new_v, new_theta)



    }
};

class Node: public Car {
    private:

    public:
}
