#ifndef MAP_VALIDATOR_HPP
#define MAP_VALIDATOR_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <memory>

namespace random_explorer {

class MapValidator {
public:
    MapValidator() = default;
    
    // Store latest costmap for collision checking
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        map_ = map;
    }
    
    // Check if point (x,y) is navigable with specified clearance radius
    // allow_unknown: treat unknown cells (-1) as valid for frontier exploration
    bool isValidPoint(double x, double y, double clearance = 0.3, bool allow_unknown = false) const {
        if (!map_) return false;
        
        const auto& info = map_->info;
        
        // Convert world (m) to grid (cells)
        int mx = static_cast<int>((x - info.origin.position.x) / info.resolution);
        int my = static_cast<int>((y - info.origin.position.y) / info.resolution);
        
        // Reject if outside map bounds
        if (mx < 0 || mx >= static_cast<int>(info.width) ||
            my < 0 || my >= static_cast<int>(info.height)) {
            return false;
        }
        
        // Check center cell: must be free (0-50) or unknown (if allowed)
        int center_idx = my * info.width + mx;
        int8_t center_val = map_->data[center_idx];
        
        if (center_val > 50) return false;  // Occupied
        if (center_val < 0 && !allow_unknown) return false;  // Unknown
        
        // Check circular clearance zone around center point
        int clearance_cells = static_cast<int>(std::ceil(clearance / info.resolution));
        double clearance_sq = clearance * clearance;
        
        for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
            for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
                // Skip cells outside clearance radius
                double dist_sq = (dx * info.resolution) * (dx * info.resolution) + 
                                 (dy * info.resolution) * (dy * info.resolution);
                if (dist_sq > clearance_sq) continue;
                
                int check_x = mx + dx;
                int check_y = my + dy;
                
                // Skip out-of-bounds cells (treat as free at map edges)
                if (check_x < 0 || check_x >= static_cast<int>(info.width) ||
                    check_y < 0 || check_y >= static_cast<int>(info.height)) {
                    continue;
                }
                
                int index = check_y * info.width + check_x;
                int8_t cell_val = map_->data[index];
                
                // Reject if occupied within clearance zone
                if (cell_val > 50) return false;
                
                // Reject unknown within clearance zone (if not allowed)
                if (cell_val < 0 && !allow_unknown) return false;
            }
        }
        return true;
    }
    
    // Get map extents in world coordinates (meters)
    bool getMapBounds(double& min_x, double& max_x, double& min_y, double& max_y) const {
        if (!map_) return false;
        
        const auto& info = map_->info;
        min_x = info.origin.position.x;
        min_y = info.origin.position.y;
        max_x = min_x + info.width * info.resolution;
        max_y = min_y + info.height * info.resolution;
        return true;
    }
    
    bool hasMap() const { return map_ != nullptr; }
    
    // Return grid resolution (m/cell), default 0.05m if no map
    double getResolution() const { 
        return map_ ? map_->info.resolution : 0.05; 
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

} // namespace random_explorer

#endif // MAP_VALIDATOR_HPP
