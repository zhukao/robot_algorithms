#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

// 定义二维点结构
struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

// 定义树节点
struct Node {
    Point position;
    std::shared_ptr<Node> parent;
    Node(Point pos, std::shared_ptr<Node> p = nullptr) : position(pos), parent(p) {}
};

// RRT规划器类
class RRTPlanner {
private:
    std::vector<std::shared_ptr<Node>> tree;
    Point start, goal;
    std::vector<cv::Rect> obstacles;
    double stepSize;
    double goalBias;
    int maxIterations;
    int width, height;
    
public:
    RRTPlanner(Point s, Point g, double step, double bias, int iter, int w, int h) 
        : start(s), goal(g), stepSize(step), goalBias(bias), maxIterations(iter), width(w), height(h) {
        tree.push_back(std::make_shared<Node>(start));
        std::srand(std::time(nullptr));
    }
    
    // 添加障碍物
    void addObstacle(cv::Rect obstacle) {
        obstacles.push_back(obstacle);
    }
    
    // 检查点是否在障碍物内
    bool isInObstacle(Point p) {
        for (auto& obs : obstacles) {
            if (p.x >= obs.x && p.x <= obs.x + obs.width &&
                p.y >= obs.y && p.y <= obs.y + obs.height) {
                return true;
            }
        }
        return false;
    }
    
    // 检查两点之间的线段是否穿过障碍物
    bool checkLineCollision(Point p1, Point p2) {
        // 简单的线段检测，可根据需要替换为更精确的算法
        int steps = 10;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Point p(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
            if (isInObstacle(p)) {
                return true;
            }
        }
        return false;
    }
    
    // 生成随机点
    Point getRandomPoint() {
        if ((double)std::rand() / RAND_MAX < goalBias) {
            return goal;
        }
        return Point(std::rand() % width, std::rand() % height);
    }
    
    // 找到树中距离给定点最近的节点
    std::shared_ptr<Node> findNearestNode(Point p) {
        std::shared_ptr<Node> nearest = tree[0];
        double minDist = std::sqrt(std::pow(p.x - nearest->position.x, 2) + 
                                   std::pow(p.y - nearest->position.y, 2));
        
        for (auto& node : tree) {
            double dist = std::sqrt(std::pow(p.x - node->position.x, 2) + 
                                   std::pow(p.y - node->position.y, 2));
            if (dist < minDist) {
                minDist = dist;
                nearest = node;
            }
        }
        return nearest;
    }
    
    // 从nearest向random扩展一步
    Point extend(std::shared_ptr<Node> nearest, Point random) {
        double dist = std::sqrt(std::pow(random.x - nearest->position.x, 2) + 
                               std::pow(random.y - nearest->position.y, 2));
        
        if (dist <= stepSize) {
            return random;
        } else {
            double theta = std::atan2(random.y - nearest->position.y, random.x - nearest->position.x);
            return Point(nearest->position.x + stepSize * std::cos(theta),
                         nearest->position.y + stepSize * std::sin(theta));
        }
    }
    
    // 检查是否达到目标
    bool reachedGoal(std::shared_ptr<Node> node) {
        double dist = std::sqrt(std::pow(node->position.x - goal.x, 2) + 
                               std::pow(node->position.y - goal.y, 2));
        return dist <= stepSize;
    }
    
    // 执行RRT规划
    std::vector<Point> plan() {
        for (int i = 0; i < maxIterations; ++i) {
            Point random = getRandomPoint();
            std::shared_ptr<Node> nearest = findNearestNode(random);
            Point newPoint = extend(nearest, random);
            
            if (!isInObstacle(newPoint) && !checkLineCollision(nearest->position, newPoint)) {
                auto newNode = std::make_shared<Node>(newPoint, nearest);
                tree.push_back(newNode);
                
                if (reachedGoal(newNode)) {
                    std::cout << "Goal reached after " << i << " iterations!" << std::endl;
                    return buildPath(newNode);
                }
            }
        }
        
        std::cout << "Failed to reach goal after " << maxIterations << " iterations." << std::endl;
        // 返回最近的路径
        std::shared_ptr<Node> nearestToGoal = tree[0];
        double minDist = std::sqrt(std::pow(goal.x - tree[0]->position.x, 2) + 
                                  std::pow(goal.y - tree[0]->position.y, 2));
        
        for (auto& node : tree) {
            double dist = std::sqrt(std::pow(goal.x - node->position.x, 2) + 
                                  std::pow(goal.y - node->position.y, 2));
            if (dist < minDist) {
                minDist = dist;
                nearestToGoal = node;
            }
        }
        
        return buildPath(nearestToGoal);
    }
    
    // 构建从起点到指定节点的路径
    std::vector<Point> buildPath(std::shared_ptr<Node> node) {
        std::vector<Point> path;
        std::shared_ptr<Node> current = node;
        
        while (current != nullptr) {
            path.push_back(current->position);
            current = current->parent;
        }
        
        // 反转路径，使其从起点到终点
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    // 获取树的所有节点
    const std::vector<std::shared_ptr<Node>>& getTree() const {
        return tree;
    }
};

int main() {
    // 初始化窗口
    const int width = 800;
    const int height = 600;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    
    // 初始化RRT规划器
    Point start(50, height/2);
    Point goal(width - 50, height/2);
    RRTPlanner planner(start, goal, 10.0, 0.05, 5000, width, height);
    
    // 添加障碍物
    planner.addObstacle(cv::Rect(300, 200, 200, 50));
    planner.addObstacle(cv::Rect(450, 300, 50, 150));
    planner.addObstacle(cv::Rect(200, 400, 150, 50));
    
    // 路径规划标志
    // bool planningStarted = true;
    // bool planningFinished = false;
    std::vector<Point> path;
    
    // 创建窗口
    cv::namedWindow("RRT Path Planning", cv::WINDOW_NORMAL);
    cv::resizeWindow("RRT Path Planning", width, height);
    
    // 主循环
    // while (true) 
    {
        // 清空图像
        image = cv::Scalar(0, 0, 0);
        
        // 绘制障碍物
        for (auto& obs : planner.getTree()) {
            cv::rectangle(image, cv::Rect(300, 200, 200, 50), cv::Scalar(0, 0, 255), -1);
            cv::rectangle(image, cv::Rect(450, 300, 50, 150), cv::Scalar(0, 0, 255), -1);
            cv::rectangle(image, cv::Rect(200, 400, 150, 50), cv::Scalar(0, 0, 255), -1);
        }
        
        // 绘制起点和终点
        cv::circle(image, cv::Point(start.x, start.y), 8, cv::Scalar(0, 255, 0), -1);
        cv::circle(image, cv::Point(goal.x, goal.y), 8, cv::Scalar(255, 0, 0), -1);
        
        // 绘制树
        const auto& tree = planner.getTree();
        for (size_t i = 1; i < tree.size(); ++i) {
            if (tree[i]->parent) {
                cv::line(image, 
                         cv::Point(tree[i]->position.x, tree[i]->position.y),
                         cv::Point(tree[i]->parent->position.x, tree[i]->parent->position.y),
                         cv::Scalar(255, 255, 0), 1);
            }
        }
        
        // 执行规划
        // if (planningStarted && !planningFinished) 
        {
            path = planner.plan();
            // planningFinished = true;
            std::cout << "Planning complete!" << std::endl;
        }
        
        // 绘制路径
        if (
          // planningFinished && 
          !path.empty()) {
            for (size_t i = 0; i < path.size() - 1; ++i) {
                cv::line(image, 
                         cv::Point(path[i].x, path[i].y),
                         cv::Point(path[i+1].x, path[i+1].y),
                         cv::Scalar(0, 255, 255), 2);
            }
        }
        
        // 显示状态文本
        std::string status = "";
        // planningFinished ? "Planning complete! (Press ESC to exit)" : 
        //                       planningStarted ? "Planning in progress..." : "Press SPACE to start planning";
        cv::putText(image, status, cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // 显示图像
        cv::imshow("RRT Path Planning", image);
        
        // 处理按键事件
        int key = cv::waitKey(0);
        // int key = cv::waitKey(10);
        // if (key == 27) {  // ESC键退出
        //     break;
        // } else if (key == 32 && !planningStarted) {  // 空格键开始规划
        //     planningStarted = true;
        // }
    }
    
    return 0;
}    