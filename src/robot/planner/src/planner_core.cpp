#include "planner_core.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

CellIndex PlannerCore::worldToGrid(double x, double y, double resolution, double origin_x, double origin_y) const {
    return CellIndex((x-origin_x)/resolution, (y-origin_y)/resolution);
}

std::pair<double, double> PlannerCore::gridToWorld(const CellIndex& idx, double resolution, double origin_x, double origin_y) const {
    return {idx.x*resolution+origin_x, idx.y*resolution+origin_y};
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
    return sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y));
}

bool PlannerCore::isFree(const CellIndex& idx, int width, int height, const std::vector<int8_t>& grid) const {
    return idx.x >= 0 && idx.x < width && idx.y >= 0 && idx.y < height && grid[idx.y*width+idx.x] < occupancy_threshold_;
}

bool PlannerCore::planPath(
    const std::vector<int8_t>& grid, int width, int height,
    double resolution, double origin_x, double origin_y,
    double start_x, double start_y,
    double goal_x, double goal_y,
    std::vector<std::pair<double, double>>& path_out) {

    path_out.clear();
    CellIndex start = worldToGrid(start_x, start_y, resolution, origin_x, origin_y);
    CellIndex goal = worldToGrid(goal_x, goal_y, resolution, origin_x, origin_y);

    if(!isFree(start, width, height, grid) || !isFree(goal, width, height, grid)) return false;

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    g_score[start] = 0.0;
    open_set.emplace(start, heuristic(start, goal));

    while(!open_set.empty()){
        CellIndex current = open_set.top().index;
        open_set.pop();

        if(current == goal){
            std::vector<CellIndex> cell_path;
            CellIndex node = current;
            while(!(node == start)){
                cell_path.push_back(node);
                node = came_from[node];
            }
            cell_path.push_back(start);
            std::reverse(cell_path.begin(), cell_path.end());

            for(const auto& idx: cell_path){
                path_out.push_back(gridToWorld(idx, resolution, origin_x, origin_y));
            }
            return true;
        }
        else{
            for(int dx = -1; dx <= 1; dx++){
                for(int dy = -1; dy <= 1; dy++){
                    if(dx == 0 && dy == 0) continue;

                    CellIndex neighbour(current.x + dx, current.y + dy);
                    if(!isFree(neighbour, width, height, grid)) continue;

                    double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
                    double tentative_g = g_score[current] + step_cost;

                    auto it = g_score.find(neighbour);
                    if(it == g_score.end() || tentative_g < it->second){
                        came_from[neighbour] = current;
                        g_score[neighbour] = tentative_g;
                        open_set.emplace(neighbour, tentative_g + heuristic(neighbour, goal));
                    }
                }
            }
        }
    }
    return false;
}

}
