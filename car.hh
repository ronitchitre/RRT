#include "constants.hh"

float* rk4method_for_car(Car ic_car, float init_time, float end_time, int N) {
    float h = (end_time - init_time) / N;
}

class Car{
    private:
        float x;
        float y;
        float theta;
        float ster_ang = 0;
        float vel = car::car_velocity;
        float length = car::car_length;
    
    public:
    Car(float x, float y, float theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
    float* dynamics() {
        float state_dynamics[3];
        state_dynamics[0] = x * cos(theta);
        state_dynamics[1] = y * sin(theta);
        state_dynamics[2] = this->vel * tan(this->ster_ang) / this->length;
        return state_dynamics;
    }
};