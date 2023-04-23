#include "PurePursuit.h"

PurePursuit::PurePursuit(point target_coordinates[], uint target_num, double look_ahead_dist, double initial_x, double initial_y, double initial_theta, double finish_dist)
    : target_num(target_num), look_ahead_dist(abs(look_ahead_dist)), finish_dist(abs(finish_dist))
{
    x = initial_x; y = initial_y; theta = initial_theta;
    for (int i = 0; i < target_num; i++) {
        this->target_coordinates[i].x = target_coordinates[i].x;
        this->target_coordinates[i].y = target_coordinates[i].y;
    }
    for (int i = 0; i < target_num - 1; i++) {
        double dist = sqrt(pow(target_coordinates[i+1].x - target_coordinates[i].x, 2) + pow(target_coordinates[i+1].y - target_coordinates[i].y, 2));
        pass_unit[i].x = (target_coordinates[i+1].x - target_coordinates[i].x) / dist;
        pass_unit[i].y = target_coordinates[i+1].y - target_coordinates[i].y / dist;
    }
}

double PurePursuit::calc_dist(point a, point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

vec PurePursuit::calc_unit(point a, point b) {
    double dist = calc_dist(a, b);
    return { (b.x - a.x) / dist, (b.y - a.y) / dist };
}

double PurePursuit::calc_angle(point current, point target1, point target2) {
    vec unit1 = calc_unit(current, target1);
    vec unit2 = calc_unit(current, target2);
    return acos(unit1.x * unit2.x + unit1.y * unit2.y);
}

double PurePursuit::perpendicular_dist(point current, uint target_index, point &intersection) {
    double a = pass_unit[target_index].y;
    double b = -pass_unit[target_index].x;
    double c = -a * target_coordinates[target_index].x - b * target_coordinates[target_index].y;
    double d = fabs(a * current.x + b * current.y + c) / sqrt(a * a + b * b);
    intersection.x = b*b * current.x - a*b * current.y - a*c / (a*a + b*b);
    intersection.y = a*a * current.y - a*b * current.x - b*c / (a*a + b*b);
    return d;
}

bool PurePursuit::is_inside(uint target_index, point intersection) {
    double angle = calc_angle(intersection, target_coordinates[target_index], target_coordinates[target_index + 1]);
    return angle > M_PI_2;
}

double PurePursuit::compute_angvel(double x, double y, double theta) {
    // 最後の線分の端点に到達したら、angvelを0にする
    if (calc_dist({x, y}, target_coordinates[target_num - 1]) < finish_dist) {
        finish_flag = true;
        return 0.0;
    }

    // 現在位置と最も距離の短い線分とその交点を求める
    int min_num = -1; double min_dist = MAXFLOAT; point min_intersection;
    for (int i = 0; i < target_num - 1; i++) {
        point intersection;
        // i番目の線分と現在位置との距離を求める
        double dist = perpendicular_dist({x, y}, i, intersection);
        // 最小距離を更新する
        if (dist < min_dist) {
            // 垂線の足が線分の端点の内側にある場合
            if (is_inside(i, intersection)) {
                min_num = i;
                min_dist = dist;
                min_intersection = intersection;
            }
        }
    }

    // 求めた距離とlook_ahead_distの三平方の定理を用いて、lookaheadpointを求める
    point lookaheadpoint;
    if (min_num == -1) {
        // 線分の端点の内側にない場合は、最も近い線分の端点をlookaheadpointとする
        double dist1 = calc_dist({x, y}, target_coordinates[0]);
        double dist2 = calc_dist({x, y}, target_coordinates[target_num - 1]);
        if (dist1 < dist2) {
            lookaheadpoint = target_coordinates[0];
        } else {
            lookaheadpoint = target_coordinates[target_num - 1];
        }
    } else {
        // 線分の端点の内側にある場合は、垂線の足を通る直線とlook_ahead_distの三平方の定理を用いて、lookaheadpointを求める
        lookaheadpoint.x = min_intersection.x + sqrt(pow(look_ahead_dist, 2) - pow(min_dist, 2)) * pass_unit[min_num].x;
        lookaheadpoint.y = min_intersection.y + sqrt(pow(look_ahead_dist, 2) - pow(min_dist, 2)) * pass_unit[min_num].y;

        // lookaheadpointが線分の内側にない場合は、次の線分の垂線の足を通る直線とlook_ahead_distの三平方の定理を用いて、lookaheadpointを求める
        if (!is_inside(min_num, lookaheadpoint)) {
            // あとで書く
        }
    }

    // 現在位置とlookaheadpointから方位誤差を求める。
    double angvel = atan2(lookaheadpoint.y - y, lookaheadpoint.x - x) - theta;

    return angvel;
}