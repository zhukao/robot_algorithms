#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <algorithm>
#include <opencv2/opencv.hpp>

struct Node {
    int x, y;
    int g, h; // g: cost from start, h: heuristic to goal
    bool operator==(const Node& other) const { return x == other.x && y == other.y; }
    
    struct Hash {
        size_t operator()(const Node& n) const {
            return std::hash<int>()(n.x ^ (n.y << 16));
            // return std::hash<std::string>()(std::to_string(n.x) + std::to_string(n.y));
        }
    };
};

struct SearchingProcess {
  std::vector<std::pair<int, int>> searched_nodes;

};

// Heuristic function using Manhattan distance
int heuristic(const Node& a, const Node& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// A* algorithm
std::vector<std::pair<int, int>> aStar(
    const std::vector<std::vector<int>>& grid,
    std::pair<int, int> start,
    std::pair<int, int> goal,
    SearchingProcess &searching_process
) {
    std::cout << "Run a star, start: " << start.first << ", " << start.second
    << ", goal: " << goal.first << ", " << goal.second
    << "\n";

    int rows = grid.size();
    int cols = grid[0].size();
    std::vector<std::vector<Node>> cameFrom(rows, std::vector<Node>(cols));

    int cellSize = std::min(1920 / rows, 1080 / cols);
    cv::Mat image(rows * cellSize, cols * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    std::priority_queue< Node, std::vector<Node>,
        std::function<bool(const Node&, const Node&)> > openList(
      [](const Node& a, const Node& b) {
            return (a.g + a.h) > (b.g + b.h);
        });
    
    std::unordered_set<Node, Node::Hash> closedList;

    Node startNode{start.first, start.second, 0, 0};
    openList.push(startNode);

    {
      cv::Point center(start.second * cellSize + cellSize / 2,
        start.first * cellSize + cellSize / 2);
      cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
    }
    {
      cv::Point center(goal.second * cellSize + cellSize / 2,
        goal.first * cellSize + cellSize / 2);
      cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
    }

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

    std::string status = "Press SPACE to start planning, ESC to exit";
    cv::putText(image, status, cv::Point(cellSize * 0.1, cellSize * 0.5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.02 * cellSize, cv::Scalar(255, 0, 255), 2);

    while (true) {
      cv::imshow("A* Path", image);
      int key = cv::waitKey(10);
      if (key == 27) {  // ESC键退出
          return std::vector<std::pair<int, int>>{};
      } else if (key == 32) {  // 空格键开始规划
          break;
      }
    }

    // For neighbors exploration
    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};
    
    while (!openList.empty()) {
      Node current = openList.top();
      openList.pop();

      if (current.x == goal.first && current.y == goal.second) {
        // Reconstruct path
        std::vector<std::pair<int, int>> path;
        while (current.x != startNode.x || current.y != startNode.y) {
            path.emplace_back(current.x, current.y);
        
            cv::Point center(current.y * cellSize + cellSize / 2,
              current.x * cellSize + cellSize / 2);
            cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
            cv::imshow("A* Path", image);
            cv::waitKey(20);

            current = cameFrom[current.x][current.y];
        }
        path.emplace_back(startNode.x, startNode.y);
        cv::Point center(startNode.y * cellSize + cellSize / 2,
          startNode.x * cellSize + cellSize / 2);
        cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
        cv::imshow("A* Path", image);
        cv::waitKey(5);

        std::reverse(path.begin(), path.end());
        printf("Searching success, path size: %ld\n", path.size());
        return path;
      }

      closedList.insert(current);

      for (int i = 0; i < 4; ++i) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];

        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] != 0) {
          Node neighbor{nx, ny, current.g + 1, heuristic({nx, ny}, {goal.first, goal.second})};
          if (closedList.find(neighbor) != closedList.end()) {
            continue;
          } else {
            openList.push(neighbor);
            // printf("open list push (%d, %d)\n", nx, ny);
            cameFrom[nx][ny] = current;
            searching_process.searched_nodes.push_back({nx, ny});

            static int search_idx = 1;
            std::string text = std::to_string(search_idx++);
            cv::putText(image, text,
              cv::Point(ny * cellSize + cellSize * 0.05,
              nx * cellSize + cellSize * 0.75),
              cv::FONT_HERSHEY_SIMPLEX, 0.02 * cellSize, cv::Scalar(0, 255, 0), 1);
            cv::imshow("A* Path", image);
            cv::waitKey(5);
          }
        }
      }
    }

    return {}; // No path found
}

void visualizePath(const std::vector<std::vector<int>>& grid,
  const std::vector<std::pair<int, int>>& path,
  const SearchingProcess& searching_process) {
    int rows = grid.size();
    int cols = grid[0].size();
    int cellSize = std::min(1920 / rows, 1080 / cols);
    cv::Mat image(rows * cellSize, cols * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

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

    // Draw path
    for (const auto& p : path) {
      cv::Point center(p.second * cellSize + cellSize / 2, p.first * cellSize + cellSize / 2);
      cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
    }

    for (size_t search_idx = 0; search_idx < searching_process.searched_nodes.size(); search_idx++) {
      std::string text = std::to_string(search_idx + 1);
      cv::putText(image, text,
        cv::Point(searching_process.searched_nodes[search_idx].second * cellSize + cellSize * 0.05,
          searching_process.searched_nodes[search_idx].first * cellSize + cellSize * 0.75),
        cv::FONT_HERSHEY_SIMPLEX, 0.02 * cellSize, cv::Scalar(0, 255, 0), 1);
    }

    // Show image
    cv::imshow("A* Path", image);
    cv::waitKey(0);
}

int main(int argc, char ** argv) {
    std::string usage = "Usage: ./bin [row] [col] [try count]\n";
    printf("%s\n", usage.data());

    int row = 10;
    int col = 30;
    int try_count = 50;

    if (argc >= 3)  {
      row = atoi(argv[1]);
      col = atoi(argv[2]);
      if (argc == 4) {
        try_count = atoi(argv[3]);
      }
    } else {
      // printf("%s\n", usage.data());
      // return -1;
    }
    if (row > 0 && row <= 1000 && col > 0 && col <= 1000 && try_count > 0 && try_count <= 100) {
      printf("row: %d, col: %d, try_count: %d\n", row, col, try_count);
    } else {
      printf("Invalid inputs. row: %d, col: %d, try_count: %d\n", row, col, try_count);
      return -1;
    }
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    while (try_count-- > 0) {
      printf("try_count left: %d\n", try_count);
      std::vector<std::vector<int>> grid;
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
      grid[0][0] = 1;
      grid[idx_end_x][idx_end_y] = 1;

      SearchingProcess searching_process;
      auto path = aStar(grid, {0, 0}, {idx_end_x, idx_end_y},
        searching_process);
      if (!path.empty()) {
        cv::waitKey(0);
        return 0;
      } else {
        std::cout << "No path found.\n";
        continue;
      }
    }

    return 0;
}