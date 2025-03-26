#include <cmath>
#include <vector>
#include <array>
#include <iostream>

#define DISTANCE_AROUND_BUOYS 2.0

class GoalPointsGenerator {
    public:
    std::array<std::array<double, 2>, 6> get_points(double top_buoy_x, double top_buoy_y, double bottom_buoy_x, double bottom_buoy_y) {
        double x_distance = top_buoy_x - bottom_buoy_x;
        double y_distance = top_buoy_y - bottom_buoy_y;

        // Handle division by zero
        if (x_distance == 0.0) {
            std::cerr << "Error: x_distance is zero, cannot calculate slope." << std::endl;
            return {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}};
        }

        double slope = y_distance / x_distance;
        double slope_inverse = -(1.0 / slope);
    
        double offset_up = std::sqrt((DISTANCE_AROUND_BUOYS * DISTANCE_AROUND_BUOYS) / (1 + (slope * slope)));
        double offset_sides = std::sqrt((DISTANCE_AROUND_BUOYS * DISTANCE_AROUND_BUOYS) / (1 + (1 / (slope * slope))));
    
        double tp_up_x, tp_up_y, bp_up_x, bp_up_y;
        if  (top_buoy_x > bottom_buoy_x) {
            tp_up_x = top_buoy_x + offset_up;
            tp_up_y = top_buoy_y + offset_up * slope;
            bp_up_x = bottom_buoy_x - offset_up;
            bp_up_y = bottom_buoy_y - offset_up * slope;
        } else {
            tp_up_x = top_buoy_x - offset_up;
            tp_up_y = top_buoy_y - offset_up * slope;
            bp_up_x = bottom_buoy_x + offset_up;
            bp_up_y = bottom_buoy_y + offset_up * slope;
        }
    
        double tp_left_x = top_buoy_x + offset_sides;
        double tp_left_y = top_buoy_y + offset_sides * slope_inverse;
    
        double tp_right_x = top_buoy_x - offset_sides;
        double tp_right_y = top_buoy_y - offset_sides * slope_inverse;
    
        double bp_left_x = bottom_buoy_x + offset_sides;
        double bp_left_y = bottom_buoy_y + offset_sides * slope_inverse;
    
        double bp_right_x = bottom_buoy_x - offset_sides;
        double bp_right_y = bottom_buoy_y - offset_sides * slope_inverse;
    
        std::array<std::array<double, 2>, 6> points = {{
            {tp_up_x, tp_up_y},
            {tp_left_x, tp_left_y},
            {tp_right_x, tp_right_y},
            {bp_up_x, bp_up_y},
            {bp_left_x, bp_left_y},
            {bp_right_x, bp_right_y}
        }};
    
        return points;  
    }
    
    
    double get_magnitude(double given_vector[2]){
        return std::sqrt(given_vector[0] * given_vector[0] + given_vector[1] * given_vector[1]);
    }
    
    
    double find_angle(double global_goal[2],double boat_cords[2], double current_heading[2]){
        double vector_goal[2] = {global_goal[0] - boat_cords[0], global_goal[1] - boat_cords[1]};
        double dot = vector_goal[0] * current_heading[0] + vector_goal[1] * current_heading[1];
        double magnitude = get_magnitude(vector_goal) * get_magnitude(current_heading);
        double angle = acos(dot / magnitude);
        return angle;
    }
};
