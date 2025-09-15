#ifndef MAP_VALIDATOR_HPP
#define MAP_VALIDATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>

namespace random_explorer {

class MapValidator {
public:
    MapValidator() = default;
    
    // Update the internal map representation
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        map_ = map;
    }
    
    // Check if a point is valid (not in obstacle or unknown space)
    bool isValidPoint(double x, double y, double clearance = 0.2) const {
        if (!map_) return false;
        
        // Convert world coordinates to map coordinates
        int mx = static_cast<int>((x - map_->info.origin.position.x) / map_->info.resolution);
        int my = static_cast<int>((y - map_->info.origin.position.y) / map_->info.resolution);
        
        // Check bounds
        if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
            my < 0 || my >= static_cast<int>(map_->info.height)) {
            return false;
        }
        
        // Check clearance around the point
        int clearance_cells = static_cast<int>(clearance / map_->info.resolution);
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
            for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
                int check_x = mx + dx;
                int check_y = my + dy;
                
                if (check_x >= 0 && check_x < static_cast<int>(map_->info.width) &&
                    check_y >= 0 && check_y < static_cast<int>(map_->info.height)) {
                    int index = check_y * map_->info.width + check_x;
                    // Occupied (>50) or unknown (-1) cells are invalid
                    if (map_->data[index] > 50 || map_->data[index] < 0) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    bool hasMap() const { return map_ != nullptr; }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

} // namespace random_explorer

#endif // MAP_VALIDATOR_HPP