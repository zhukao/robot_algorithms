#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

struct SearchingNode {
    int x, y;
    int g, h; // g: cost from start, h: heuristic to goal
    bool operator==(const SearchingNode& other) const { return x == other.x && y == other.y; }
    
    struct Hash {
        size_t operator()(const SearchingNode& n) const {
            return std::hash<int>()(n.x ^ (n.y << 16));
            // return std::hash<std::string>()(std::to_string(n.x) + std::to_string(n.y));
        }
    };
};

struct SearchingProcess {
  std::vector<std::pair<int, int>> searched_nodes;

};

class AStarPlannerNode : public rclcpp::Node  {
public:
  AStarPlannerNode() : Node("astar_planner") {
    row = this->declare_parameter("row", row);
    col = this->declare_parameter("col", col);
    vis_interval = this->declare_parameter("vis_interval", 10);
    
    RCLCPP_WARN(this->get_logger(),
      "\n vis_interval(step mode is activatied if <=0): %d ms "\
      "\n row: %d"\
      "\n col: %d",
      vis_interval, row, col);

    if (vis_interval <= 0) {
        step_mode = true;
        vis_interval = 10;
        RCLCPP_WARN(rclcpp::get_logger("rrt_node"),
          "Set vis_interval to %d, step mode enabled. Press SPACE to continue", vis_interval);
    }

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // 生成随机数作为地图
    for (int i = 0; i < row; i++) {
      std::vector<int> temp;
      for (int j = 0; j < col; j++) {
        temp.push_back(std::rand() % (std::max(5, std::min(row, col) / 8)));
      }
      grid.push_back(temp);
    }
    
    int idx_end_x = grid.size() - 1;
    int idx_end_y = grid.at(0).size() - 1;
    // 0 is obstacle, others are free
    grid[1][0] = 1;
    grid[idx_end_x][idx_end_y] = 1;
    
    start = std::pair<int, int>{1, 0};
    goal = std::pair<int, int>{idx_end_x, idx_end_y};
  }

  std::vector<std::pair<int, int>> aStar();

private:
  int vis_interval = 10;
  int row = 10;
  int col = 30;
  std::pair<int, int> start;
  std::pair<int, int> goal;
  std::vector<std::vector<int>> grid;
  bool step_mode = false;

  int heuristic(const SearchingNode& a, const SearchingNode& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

  void WaitAction(bool& planningContinued) {
    if (!step_mode) {
      cv::waitKey(vis_interval);
      return;
    }
    planningContinued = false;
    while (rclcpp::ok()) {
      int key = cv::waitKey(vis_interval);
      if (key == 27) {  // ESC键退出
        rclcpp::shutdown(); 
      } else if (key == 32) {  // 空格键
        planningContinued = !planningContinued;
      } 
      if (!planningContinued) {
        continue;
      } else {
        break;
      }
    }
  }
};

// Heuristic function using Manhattan distance


// A* algorithm
std::vector<std::pair<int, int>> AStarPlannerNode::aStar() {
    std::cout << "Run a star, start: " << start.first << ", " << start.second
    << ", goal: " << goal.first << ", " << goal.second
    << "\n";

    int rows = grid.size();
    int cols = grid[0].size();
    std::vector<std::vector<SearchingNode>> cameFrom(rows, std::vector<SearchingNode>(cols));

    int width = 1280;
    int height = 640;
    int cellSize = std::min(width / rows, height / cols);
    cv::Mat image(rows * cellSize, cols * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // 创建窗口
    // 使窗口可调整大小
    cv::namedWindow("A* Path", cv::WINDOW_NORMAL);
    // 调整窗口大小
    cv::resizeWindow("A* Path", width, height);
    cv::imshow("A* Path", image);
    cv::waitKey(5);

    std::priority_queue< SearchingNode, std::vector<SearchingNode>,
        std::function<bool(const SearchingNode&, const SearchingNode&)> > openList(
      [](const SearchingNode& a, const SearchingNode& b) {
            return (a.g + a.h) > (b.g + b.h);
        });
    
    std::unordered_set<SearchingNode, SearchingNode::Hash> closedList;

    SearchingNode startNode{start.first, start.second, 0, 0};
    openList.push(startNode);

    // Draw start and goal
    cv::circle(image, 
      cv::Point(start.second * cellSize + cellSize / 2,
      start.first * cellSize + cellSize / 2),
      cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
    cv::circle(image, 
      cv::Point(goal.second * cellSize + cellSize / 2,
      goal.first * cellSize + cellSize / 2),
      cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)

    // Draw grid
    for (int x = 0; x < rows; ++x) {
      for (int y = 0; y < cols; ++y) {
        cv::Rect rect(y * cellSize, x * cellSize, cellSize, cellSize);
        if (grid[x][y] == 0)
            cv::rectangle(image, rect, cv::Scalar(0, 0, 0), -1); // Obstacle (black)
        else
            cv::rectangle(image, rect, cv::Scalar(200, 200, 200), 1); // Free space (gray border)
      }
    }

    std::string status = "Press SPACE to start/pause planning, ESC to exit";
    cv::putText(image, status, cv::Point(cellSize * 0.1, cellSize * 0.75), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
    cv::imshow("A* Path", image);

    // For neighbors exploration
    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};
    
    while (rclcpp::ok()) {
      int key = cv::waitKey(vis_interval);
      if (key == 27) {  // ESC键退出
        rclcpp::shutdown(); 
      } else if (key == 32) {  // 空格键
        break;
      }
    }

    bool planningContinued = false;
    while (rclcpp::ok() && !openList.empty()) {
      SearchingNode current = openList.top();
      openList.pop();

      if (current.x == goal.first && current.y == goal.second) {
        // Reconstruct path
        std::vector<std::pair<int, int>> path;
        while (rclcpp::ok() && current.x != startNode.x || current.y != startNode.y) {
            path.emplace_back(current.x, current.y);
        
            cv::Point center(current.y * cellSize + cellSize / 2,
              current.x * cellSize + cellSize / 2);
            cv::circle(image, center, cellSize * 0.2, cv::Scalar(0, 0, 255), -1); // Path (red)
            cv::imshow("A* Path", image);
            WaitAction(planningContinued);
            current = cameFrom[current.x][current.y];
        }
        path.emplace_back(startNode.x, startNode.y);
        cv::Point center(startNode.y * cellSize + cellSize / 2,
          startNode.x * cellSize + cellSize / 2);
        cv::circle(image, center, cellSize * 0.3, cv::Scalar(0, 0, 255), -1); // Path (red)
        cv::imshow("A* Path", image);
        WaitAction(planningContinued);
        std::reverse(path.begin(), path.end());
        printf("Searching success, path size: %ld\n", path.size());
        return path;
      }

      closedList.insert(current);

      for (int i = 0; i < 4; ++i) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];

        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] != 0) {
          SearchingNode neighbor{nx, ny, current.g + 1, heuristic({nx, ny}, {goal.first, goal.second})};
          if (closedList.find(neighbor) != closedList.end()) {
            continue;
          } else {
            openList.push(neighbor);
            // printf("open list push (%d, %d)\n", nx, ny);
            cameFrom[nx][ny] = current;

            static int search_idx = 1;
            std::string text = std::to_string(search_idx++);
            cv::putText(image, text,
              cv::Point(ny * cellSize + cellSize * 0.05,
              nx * cellSize + cellSize * 0.75),
              cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            cv::imshow("A* Path", image);
            WaitAction(planningContinued);
          }
        }
      }
    }

    return {}; // No path found
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  int try_count = 10;
  while (rclcpp::ok() && try_count-- > 0) {
    printf("try_count left: %d\n", try_count);
    
    AStarPlannerNode astar_node;
if (!astar_node.aStar().empty()) {
      while (rclcpp::ok()) {
        cv::waitKey(10);
      }
      return 0;
    } else {
      std::cout << "No path found.\n";
      continue;
    }
  }

  rclcpp::shutdown();
  return 0;
}