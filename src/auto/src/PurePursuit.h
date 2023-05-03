#ifndef PurePursuit_h
#define PurePursuit_h

#include <math.h>
#include <iostream>
using namespace std;

#define ROOT3_2 0.86602540378443864676372317075294 // √3/2
#define ROOT2 1.4142135623730950488016887242097 // √2
#define M_PI_6 0.52359877559829887307710723054658 // π/6

#include "visualization_msgs/MarkerArray.h"

struct point {
    double x, y;
};

struct vec {
    double x, y;
};

class PurePursuit {
private:
    bool finish_flag = false;
    double x = 0.0, y = 0.0, theta = 0.0;
    point target_coordinates[100];
    vec pass_unit[100];
    point near_point, look_ahead_point;
    const uint target_num;
    double look_ahead_dist;
    const double finish_dist;
    double calc_dist(point a, point b);
    vec calc_unit(point a, point b);
    double calc_angle(point current, point target1, point target2);
    bool is_inside(point intersection, uint target_index);
    double perpendicular_dist(point current, uint target_index, point &intersection);
public:
    PurePursuit(point target_coordinates[], uint target_num, double look_ahead_dist, double finish_dist = 0.1);
    double compute_angerr(double x, double y, double theta);
    void set_look_ahead_dist(double look_ahead_dist) {this->look_ahead_dist = abs(look_ahead_dist);}
    bool get_finish_flag() {return finish_flag;}
    visualization_msgs::MarkerArray get_marker(string frame_id);

    // double calc_dist(point a, point b);
    // vec calc_unit(point a, point b);
    // double calc_angle(point current, point target1, point target2);
    // bool is_inside(point intersection, uint target_index);
    // double perpendicular_dist(point current, uint target_index, point &intersection);
};

#endif