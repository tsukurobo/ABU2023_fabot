#include "PurePursuit.h"

PurePursuit::PurePursuit(point target_coordinates[], uint target_num, double look_ahead_dist, double finish_dist)
    : target_num(target_num), look_ahead_dist(abs(look_ahead_dist)), finish_dist(abs(finish_dist))
{
    for (int i = 0; i < target_num; i++) {
        this->target_coordinates[i] = target_coordinates[i];
    }
    for (int i = 0; i < target_num - 1; i++) {
        pass_unit[i] = calc_unit(target_coordinates[i], target_coordinates[i + 1]);
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

bool PurePursuit::is_inside(point intersection, uint target_index) {
    double angle = calc_angle(intersection, target_coordinates[target_index], target_coordinates[target_index + 1]);
    return angle > M_PI_2;
}

double PurePursuit::compute_angerr(double x, double y, double theta) {
    // 最後の線分の端点に到達したら、最終点との方位誤差を返す
    if (calc_dist({x, y}, target_coordinates[target_num - 1]) < finish_dist) {
        finish_flag = true;
        double angerr = atan2(target_coordinates[target_num - 1].y - y, target_coordinates[target_num - 1].x - x) - theta;
        return angerr;
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
            if (is_inside(intersection, i)) {
                min_num = i;
                min_dist = dist;
                min_intersection = intersection;
            }
        }
    }
    // cout << "min_num: " << min_num << endl;
    // cout << "min_dist: " << min_dist << endl;
    // cout << "min_intersection: " << min_intersection.x << ", " << min_intersection.y << endl;
    near_point = min_intersection;

    // look_ahead_pointを求める
    if (min_num == -1) {
        // 線分の端点の内側にない場合は、最も近い線分の端点をlook_ahead_pointとする
        double dist1 = calc_dist({x, y}, target_coordinates[0]);
        double dist2 = calc_dist({x, y}, target_coordinates[target_num - 1]);
        if (dist1 < dist2) {
            look_ahead_point = target_coordinates[0];
        } else {
            look_ahead_point = target_coordinates[target_num - 1];
        }
    } 
    else {
        // 線分の端点の内側にある場合は、垂線の交点からlook_ahead_distだけ進んだところをlook_ahead_pointとする
        look_ahead_point.x = min_intersection.x + look_ahead_dist * pass_unit[min_num].x;
        look_ahead_point.y = min_intersection.y + look_ahead_dist * pass_unit[min_num].y;

        // look_ahead_pointが線分の内側にない場合は、線分の終点とlook_ahead_pointの距離だけ次の線分に沿って進め、look_ahead_pointを求める
        if (!is_inside(look_ahead_point, min_num)) {
            double dist = calc_dist(look_ahead_point, target_coordinates[min_num + 1]);
            look_ahead_point.x = target_coordinates[min_num + 1].x + dist * pass_unit[min_num + 1].x;
            look_ahead_point.y = target_coordinates[min_num + 1].y + dist * pass_unit[min_num + 1].y;
        }
    }
    // cout << "look_ahead_point: " << look_ahead_point.x << ", " << look_ahead_point.y << endl;

    // 現在位置とlook_ahead_pointから方位誤差を求める。
    double angerr = atan2(look_ahead_point.y - y, look_ahead_point.x - x) - theta;

    return angerr;
}

visualization_msgs::MarkerArray PurePursuit::get_marker(string frame_id) {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.05;
    path_marker.color.g = path_marker.color.a = 1.0;
    for(int i = 0; i < target_num; i++) {
        geometry_msgs::Point p;
        p.x = target_coordinates[i].x;
        p.y = target_coordinates[i].y;
        path_marker.points.push_back(p);
    }
    marker_array.markers.push_back(path_marker);

    visualization_msgs::Marker near_point_marker;
    near_point_marker.header.frame_id = frame_id;
    near_point_marker.header.stamp = ros::Time::now();
    near_point_marker.ns = "near_point";
    near_point_marker.action = visualization_msgs::Marker::ADD;
    near_point_marker.type = visualization_msgs::Marker::SPHERE;
    near_point_marker.scale.x = near_point_marker.scale.y = near_point_marker.scale.z = 0.1;
    near_point_marker.color.b = near_point_marker.color.a = 1.0;
    near_point_marker.pose.position.x = near_point.x;
    near_point_marker.pose.position.y = near_point.y;
    marker_array.markers.push_back(near_point_marker);

    visualization_msgs::Marker look_ahead_point_marker;
    look_ahead_point_marker.header.frame_id = frame_id;
    look_ahead_point_marker.header.stamp = ros::Time::now();
    look_ahead_point_marker.ns = "look_ahead_point";
    look_ahead_point_marker.action = visualization_msgs::Marker::ADD;
    look_ahead_point_marker.type = visualization_msgs::Marker::SPHERE;
    look_ahead_point_marker.scale.x = look_ahead_point_marker.scale.y = look_ahead_point_marker.scale.z = 0.1;
    look_ahead_point_marker.color.r = look_ahead_point_marker.color.a = 1.0;
    look_ahead_point_marker.pose.position.x = look_ahead_point.x;
    look_ahead_point_marker.pose.position.y = look_ahead_point.y;
    marker_array.markers.push_back(look_ahead_point_marker);

    return marker_array;
}