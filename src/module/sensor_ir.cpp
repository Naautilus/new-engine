#include "sensor_ir.hpp"
#include "../physics_object/object.hpp"

// x = forward distance, y and z = perspectivified
//struct vector::scopespace : vector::localspace {};

namespace module {

signal_point::signal_point() {
    position_scopespace.distance() = 0;
    position_scopespace.scope_x() = 0;
    position_scopespace.scope_y() = 0;
    signal_strength = 0;
}

signal_point::signal_point(vector::worldspace target_position, vector::worldspace sensor_position, Eigen::Quaterniond rotation, double base_signal_strength) {
    vector::worldspace position_relative_worldspace = target_position - sensor_position;
    vector::localspace position_localspace = position_relative_worldspace.to_localspace(rotation);
    position_scopespace.distance() = position_localspace.x();
    position_scopespace.scope_x() = position_localspace.y() / position_localspace.x();
    position_scopespace.scope_y() = position_localspace.z() / position_localspace.x();
    signal_strength = base_signal_strength / (position_localspace.squaredNorm());
}

signal_point::signal_point(double distance_, double scope_x_, double scope_y_, double signal_strength_) {
    position_scopespace.distance() = distance_;
    position_scopespace.scope_x() = scope_x_;
    position_scopespace.scope_y() = scope_y_;
    signal_strength = signal_strength_;
}

std::string signal_point::str() {
    std::string output = "";
    output += position_scopespace.str();
    output += "[";
    output += signal_strength;
    output += "]";
    return output;
}

sensor_cell_grid::sensor_cell_grid(int size_, double view_cone_halfarc) {
    double view_cone = tan(view_cone_halfarc * std::numbers::pi / 180);
    size = size_;
    min = -view_cone;
    max = view_cone;
    for (int x = 0; x < size; x++) {
        std::vector<signal_point> row;
        for (int y = 0; y < size; y++) {
            double x_ = 0;
            double y_ = get_value_from_axis_index(x);
            double z_ = get_value_from_axis_index(y);
            row.push_back(signal_point(x_, y_, z_, 0.0));
        }
        points.push_back(row);
    }
}

void sensor_cell_grid::increase_signals_in_circle(double center_x, double center_y, double radius, double distance, double signal) {
    std::vector<sensor_point> indices = get_points_in_circle(center_x, center_y, radius);
    for (sensor_point p : indices) {
        points[p.x()][p.y()].signal_strength += signal;
        points[p.x()][p.y()].position_scopespace.distance() += distance * signal;
        points[p.x()][p.y()].distance_weight += signal;
    }
}

std::vector<sensor_point> sensor_cell_grid::remove_invalid_indices(std::vector<sensor_point> indices) {
    std::vector<sensor_point> output;
    for (sensor_point p : indices) {
        if (p.x() < 0) continue;
        if (p.x() >= size) continue;
        if (p.y() < 0) continue;
        if (p.y() >= size) continue;
        output.push_back(p);
    }
    return output;
}

void sensor_cell_grid::calculate_indices_in_circle(double radius) {
    std::vector<sensor_point> output;
    sensor_point origin = point_from_coordinates(0, 0);
    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            vector::scopespace p_pos = points[x][y].position_scopespace;
            if (p_pos.scope_x() * p_pos.scope_x() + p_pos.scope_y() * p_pos.scope_y() < radius * radius) {
                output.push_back(sensor_point(x, y));
            }
        }
    }
    circle_radius_indices_offsets[radius] = output;
}

std::vector<sensor_point> sensor_cell_grid::get_points_in_circle(double center_x, double center_y, double radius) {
    sensor_point shift = point_from_coordinates(center_x, center_y) - point_from_coordinates(0, 0);
    if(circle_radius_indices_offsets.find(radius) == circle_radius_indices_offsets.end()) calculate_indices_in_circle(radius);
    std::vector<sensor_point> output = circle_radius_indices_offsets[radius];
    for (sensor_point& p : output) p += shift;
    return remove_invalid_indices(output);
}

sensor_point sensor_cell_grid::point_from_coordinates(double x, double y) {
    x -= min;
    y -= min;
    x /= (max - min);
    y /= (max - min);
    x *= size;
    y *= size;
    int x_ = round(x);
    int y_ = round(y);
    sensor_point point = sensor_point{x_, y_};
    return point;
}

int sensor_cell_grid::index_from_coordinates(double x, double y) {
    x -= min;
    y -= min;
    x /= (max - min);
    y /= (max - min);
    x *= size;
    y *= size;
    int x_ = round(x);
    int y_ = round(y);
    int index = (x_ + y_*size);
    return index;
}

double sensor_cell_grid::get_value_from_axis_index(int index) {
    double value = index;
    value /= size;
    value *= (max - min);
    value += min;
    return value;
}

signal_point sensor_cell_grid::get_largest_signal() {
    double max_signal_strength = points[0][0].signal_strength;
    signal_point max_signal_point = points[0][0];
    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            if (points[x][y].signal_strength > max_signal_strength) {
                max_signal_strength = points[x][y].signal_strength;
                max_signal_point = points[x][y];
            }
        }
    }
    if (std::isnan(max_signal_strength) || max_signal_strength == 0) {
        max_signal_point = signal_point(1, 0, 0, 123);
        max_signal_point.distance_weight = 1.0;
    }
    //std::cout << "largest signal: [" << max_signal_point.position_scopespace.scope_x() << "][" << max_signal_point.position_scopespace.scope_y() << "] @ " << max_signal_strength << "\n";
    return max_signal_point;
}

double sensor_cell_grid::distance_between_grid_cells() {
    double output = points[1][0].position_scopespace.scope_x() - points[0][0].position_scopespace.scope_x();
    //std::cout << "distance_between_grid_cells: " << output << "\n";
    return output;
}

void sensor_cell_grid::print(vector::scopespace target, vector::scopespace center) {
    std::string LEVELS = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
    int target_index = index_from_coordinates(target.scope_x(), target.scope_y());
    int center_index = index_from_coordinates(center.scope_x(), center.scope_y());
    double max = std::numeric_limits<double>::min();
    double min = std::numeric_limits<double>::max();
    for (auto& row : points) {
        for (signal_point& s : row) {
            max = fmax(s.signal_strength, max);
            min = fmin(s.signal_strength, min);
        }
    }
    for (int y = size - 1; y >= 0; y--) {
        for (int x = 0; x < size; x++) {
            int index = x + y*size;
            double value = points[x][y].signal_strength;
            value -= min;
            value /= (max - min);
            value = sqrt(value);
            int level = floor(value * (LEVELS.size()-1));
            if (index == target_index) {
                std::cout << "()";
                continue;
            }
            if (index == center_index) {
                std::cout << "><";
                continue;
            }
            std::cout << LEVELS[level];
            std::cout << LEVELS[level];
        }
        std::cout << "|\n";
    }
    std::cout << "\n";
}

sensor_ir::sensor_ir(double gimbal_cone_halfarc_, double view_cone_halfarc_, double target_recognition_cone_halfarc_, Eigen::Quaterniond rotation_, vector::localspace position_, double length, double width, double health_) {
    gimbal_cone_halfarc = gimbal_cone_halfarc_;
    view_cone_halfarc = view_cone_halfarc_;
    target_recognition_cone_halfarc = target_recognition_cone_halfarc_;
    rotation = rotation_;
    position = position_;
    health = health_;
}

vector::worldspace sensor_ir::get_worldspace_position(physics_object::object* parent) {
    return position.to_worldspace_positional(parent->physics_state.rotation, parent->physics_state.position);
}

void sensor_ir::update(physics_object::object* parent) {}

void sensor_ir::update_detection(physics_object::object* parent) {
    last_detection_worldspace = current_detection_worldspace;
    current_detection_relative_worldspace = get_target_position(parent);
    current_detection_worldspace = current_detection_relative_worldspace + get_worldspace_position(parent);
    detection_velocity = (1/constants::DELTA_T) * (current_detection_worldspace - last_detection_worldspace);
}

vector::worldspace sensor_ir::get_enemy_velocity(vector::worldspace current_detection_worldspace, vector::worldspace last_detection_worldspace) {
    return (1/constants::DELTA_T) * (current_detection_worldspace - last_detection_worldspace);
}

std::vector<signal_point> sensor_ir::get_signals_from_physics_objects(physics_object::object* parent) {
    std::vector<signal_point> output;
    double max_scopespace_offset = tan(view_cone_halfarc * std::numbers::pi / 180);
//
    globals::functional_physics_objects_mutex.lock();
    auto functional_physics_objects_ = globals::functional_physics_objects;
    globals::functional_physics_objects_mutex.unlock();
    
    for (auto o : functional_physics_objects_) {
        if (o->mutex) std::lock_guard<std::mutex> lock(*o->mutex);
        if (!o->properties.functional) continue;
        signal_point s = signal_point(
            o->physics_state.position,
            get_worldspace_position(parent),
            rotation * parent->physics_state.rotation,
            o->physics_state.base_signal_strength
        );
        if (s.position_scopespace.distance() <= 0) continue;
        double scopespace_offset_squared = s.position_scopespace.scope_x() * s.position_scopespace.scope_x() + s.position_scopespace.scope_y() * s.position_scopespace.scope_y();
        if (scopespace_offset_squared > max_scopespace_offset * max_scopespace_offset) continue;
        if (!ground::line_of_sight(get_worldspace_position(parent), o->physics_state.position)) continue;
        output.push_back(s);
    }
    return output;
}

vector::scopespace sensor_ir::get_target_direction(physics_object::object* parent) {
    std::vector<signal_point> signals_unfiltered = get_signals_from_physics_objects(parent);
    /*
    std::cout << "signals_unfiltered [size " << signals_unfiltered.size() << "]:\n";
    for (signal_point& s : signals_unfiltered) {
        std::cout << s.str() << "\n";
    }
    */
    const double LAST_TARGET_BONUS = 0.4;
    sensor_cell_grid grid = sensor_cell_grid(constants::SENSOR_IR_GRID_WIDTH, view_cone_halfarc);
    double target_recognition_radius = tan(target_recognition_cone_halfarc * std::numbers::pi / 180);
    for (signal_point& p : signals_unfiltered) {
        grid.increase_signals_in_circle(p.position_scopespace.scope_x(), p.position_scopespace.scope_y(), target_recognition_radius, p.position_scopespace.distance(), p.signal_strength);
    }
    if (
        !std::isnan(last_detection_scopespace.distance()) &&
        !std::isnan(last_detection_scopespace.scope_x()) &&
        !std::isnan(last_detection_scopespace.scope_y()) &&
        !std::isnan(last_detection_signal_strength)
    ) {
        grid.increase_signals_in_circle(last_detection_scopespace.scope_x(), last_detection_scopespace.scope_y(), target_recognition_radius, last_detection_scopespace.distance(), last_detection_signal_strength);
    }
    
    globals::sensor_ir_activations_mutex.lock();
    globals::sensor_ir_activations.clear();
    for (int x = 0; x < grid.points.size(); x++) {
        std::vector<double> row;
        for (int y = 0; y < grid.points[x].size(); y++) {
            row.push_back(grid.points[x][y].signal_strength);
        }
        globals::sensor_ir_activations.push_back(row);
    }
    globals::sensor_ir_activations_mutex.unlock();
    signal_point center = grid.get_largest_signal();

    /*
    since the get_largest_signal function will only select the
    first index of the strongest signal's circle, that value
    can be ever so slightly out of the bounds of the
    target_recognition_radius due to the elements being on a
    coarse grid where values are rounded all over the place;
    increasing the target_recognition_radius by 1 grid cell
    fixes that
    */
    target_recognition_radius += grid.distance_between_grid_cells();

    //std::cout << "center (get_largest_signal): " << center.str() << "\n";
    //std::cout << "target_recognition_radius: " << target_recognition_radius << "\n";
    std::vector<int> signal_indices_in_target_recognition_circle;
    for (int i = 0; i < signals_unfiltered.size(); i++) {
        //std::cout << "  signals_unfiltered[" << i << "].position_scopespace.scope_x(): " << signals_unfiltered[i].position_scopespace.scope_x() << "\n";
        //std::cout << "  signals_unfiltered[" << i << "].position_scopespace.scope_y(): " << signals_unfiltered[i].position_scopespace.scope_y() << "\n";
        //std::cout << "  center.position_scopespace.scope_x(): " << center.position_scopespace.scope_x() << "\n";
        //std::cout << "  center.position_scopespace.scope_y(): " << center.position_scopespace.scope_y() << "\n";
        double distance_x = signals_unfiltered[i].position_scopespace.scope_x() - center.position_scopespace.scope_x();
        double distance_y = signals_unfiltered[i].position_scopespace.scope_y() - center.position_scopespace.scope_y();
        //std::cout << "  distance_x: " << distance_x << "\n";
        //std::cout << "  distance_y: " << distance_y << "\n";
        if (distance_x * distance_x + distance_y * distance_y < target_recognition_radius * target_recognition_radius) {
            //std::cout << "  pushed";
            signal_indices_in_target_recognition_circle.push_back(i);
        }
        //std::cout << "\n\n";
    }
    /*
    std::cout << "signal_indices_in_target_recognition_circle: {";
    for (int i : signal_indices_in_target_recognition_circle) {
        std::cout << i << ", ";
    }
    std::cout << "\b\b}\n";
    */
    
    signal_point signals_averaged = signal_point(0, 0, 0, 0);
    for (int i : signal_indices_in_target_recognition_circle) {
        signal_point& p = signals_unfiltered[i];
        signals_averaged.distance_weight++;
        signals_averaged.position_scopespace += p.position_scopespace;
        signals_averaged.signal_strength += p.signal_strength;
    }
    signals_averaged.position_scopespace /= signals_averaged.distance_weight;
    signals_averaged.signal_strength /= signals_averaged.distance_weight;
    vector::scopespace result = signals_averaged.position_scopespace;
    last_detection_scopespace = signals_averaged.position_scopespace;
    last_detection_signal_strength = signals_averaged.signal_strength;
    //std::cout << "get_target_direction result: " << result.str() << "\n";
    //if (globals::tick % 20 == 0) grid.print(result, center.position_scopespace);
    return result;
}

vector::worldspace sensor_ir::get_target_position(physics_object::object* parent) {
    vector::scopespace result = get_target_direction(parent);
    vector::localspace result_localspace = {
        result.distance(),
        result.scope_x() * result.distance(),
        result.scope_y() * result.distance()
    };
    return result_localspace.to_worldspace(parent->physics_state.rotation);
}

}

void physics_object::object::add_sensor_ir(module::sensor_ir s) {
	std::shared_ptr<module::module> m = std::make_shared<module::sensor_ir>(s);
	properties.modules.push_back(m);
}