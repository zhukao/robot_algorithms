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
        }
    };
};

// Heuristic function using Manhattan distance
int heuristic(const Node& a, const Node& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// A* algorithm
std::vector<std::pair<int, int>> aStar(
    const std::vector<std::vector<int>>& grid,
    std::pair<int, int> start,
    std::pair<int, int> goal
) {
    std::cout << "start: " << start.first << ", " << start.second
    << ", goal: " << goal.first << ", " << goal.second
    << "\n";

    int rows = grid.size();
    int cols = grid[0].size();

    std::priority_queue< Node, std::vector<Node>,
        std::function<bool(const Node&, const Node&)> > openList(
      [](const Node& a, const Node& b) {
            return (a.g + a.h) > (b.g + b.h);
        });
    
    std::unordered_set<Node, Node::Hash> closedList;

    Node startNode{start.first, start.second, 0, 0};
    openList.push(startNode);

    std::vector<std::vector<Node>> cameFrom(rows, std::vector<Node>(cols));

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.x == goal.first && current.y == goal.second) {
            // Reconstruct path
            std::vector<std::pair<int, int>> path;
            while (current.x != startNode.x || current.y != startNode.y) {
                path.emplace_back(current.x, current.y);
                current = cameFrom[current.x][current.y];
            }
            path.emplace_back(startNode.x, startNode.y);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closedList.insert(current);

        // Explore neighbors
        int dx[] = {-1, 0, 1, 0};
        int dy[] = {0, 1, 0, -1};

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] == 0) {
                Node neighbor{nx, ny, current.g + 1, heuristic({nx, ny}, {goal.first, goal.second})};

                if (closedList.find(neighbor) != closedList.end())
                    continue;

                openList.push(neighbor);
                cameFrom[nx][ny] = current;
            }
        }
    }

    return {}; // No path found
}

void visualizePath(const std::vector<std::vector<int>>& grid, const std::vector<std::pair<int, int>>& path) {
    int cellSize = 50;
    int rows = grid.size();
    int cols = grid[0].size();

    cv::Mat image(rows * cellSize, cols * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw grid
    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            cv::Rect rect(y * cellSize, x * cellSize, cellSize, cellSize);
            if (grid[x][y] == 1)
                cv::rectangle(image, rect, cv::Scalar(0, 0, 0), -1); // Obstacle (black)
            else
                cv::rectangle(image, rect, cv::Scalar(200, 200, 200), 1); // Free space (gray border)
        }
    }

    cv::imshow("Grid", image);

    // Draw path
    for (const auto& p : path) {
        cv::Point center(p.second * cellSize + cellSize / 2, p.first * cellSize + cellSize / 2);
        cv::circle(image, center, cellSize / 4, cv::Scalar(0, 0, 255), -1); // Path (red)
    }

    // Show image
    cv::imshow("A* Path", image);
    cv::waitKey(0);
}

int main() {
    // Example grid: 0 = free cell, 1 = obstacle
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };

    int idx_end_x = grid.size() - 1;
    int idx_end_y = grid.at(0).size() - 1;
    auto path = aStar(grid, {0, 0}, {idx_end_x, idx_end_y});

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& p : path) {
            std::cout << "(" << p.first << ", " << p.second << ")\n";
        }
        visualizePath(grid, path);
    } else {
        std::cout << "No path found.\n";
    }

    return 0;
}