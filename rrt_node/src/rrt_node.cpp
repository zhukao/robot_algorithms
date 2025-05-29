#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

// 定义二维点结构
struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

// 定义树节点
struct TreeNode {
    Point position;
    std::shared_ptr<TreeNode> parent;
    TreeNode(Point pos, std::shared_ptr<TreeNode> p = nullptr) : position(pos), parent(p) {}
};

// RRT规划器类
class RRTPlanner : public rclcpp::Node {
public:
  RRTPlanner(int width_, int height_, Point start_, Point goal_, std::string node_name = "rrt_node") 
      : rclcpp::Node(node_name) {
    width = width_;
    height = height_;
    start = start_;
    goal = goal_;

    vis_interval = this->declare_parameter("vis_interval", 10);
    stepSize = this->declare_parameter("stepSize", stepSize);
    
    RCLCPP_WARN(this->get_logger(),
      "\n vis_interval(step mode is activatied if <=0): %d ms "\
      "\n stepSize: %.2f",
      vis_interval, stepSize);

    tree.push_back(std::make_shared<TreeNode>(start));
    std::srand(std::time(nullptr));
  }

  ~RRTPlanner() {
    cv::destroyAllWindows();
  }
  
private:
    int width = 800;
    int height = 600;
    std::vector<std::shared_ptr<TreeNode>> tree;
    Point start, goal;
    std::vector<cv::Rect> obstacles;
    double stepSize = 10;
    double goalBias = 0.05;
    int maxIterations = 5000;
    int vis_interval = 10;

public:  
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
    std::shared_ptr<TreeNode> findNearestNode(Point p) {
        std::shared_ptr<TreeNode> nearest = tree[0];
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
    Point extend(std::shared_ptr<TreeNode> nearest, Point random) {
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
    bool reachedGoal(std::shared_ptr<TreeNode> node) {
        double dist = std::sqrt(std::pow(node->position.x - goal.x, 2) + 
                               std::pow(node->position.y - goal.y, 2));
        return dist <= stepSize;
    }
    
    // 执行RRT规划
    std::vector<Point> plan(cv::Mat& image) {
        bool planningContinued = true;
        int vis_interval_ms = vis_interval;
        bool step_mode = false;
        if (vis_interval <= 0) {
            step_mode = true;
            vis_interval_ms = 10;
            RCLCPP_WARN(rclcpp::get_logger("rrt_node"),
              "Input vis_interval is %d, step mode enabled. Press SPACE to continue", vis_interval);
        }

        int iter = 0;
        std::shared_ptr<cv::Point> last_random_pt = nullptr;
        while (rclcpp::ok()) {
          if (iter >= maxIterations) {
            break;
          }

          int key = cv::waitKey(vis_interval_ms);
          if (key == 27) {  // ESC键退出
              rclcpp::shutdown();
          } else if (key == 32) {  // 空格键
              planningContinued = !planningContinued;
          } 
          if (!planningContinued) {
              continue;
          }
          iter++;
          Point random = getRandomPoint();
          std::shared_ptr<TreeNode> nearest = findNearestNode(random);
          Point newPoint = extend(nearest, random);
          
          cv::circle(image, cv::Point(random.x, random.y), 3, cv::Scalar(0, 0, 255), -1);
          if (!last_random_pt) {
            last_random_pt = std::make_shared<cv::Point>(random.x, random.y);
          } else {
            cv::circle(image, *last_random_pt, 3, cv::Scalar(0, 0, 128), -1);
            last_random_pt->x = random.x;
            last_random_pt->y = random.y;
          }
          cv::circle(image, cv::Point(nearest->position.x, nearest->position.y), 3, cv::Scalar(255, 255, 255), -1);
          // cv::circle(image, cv::Point(newPoint.x, newPoint.y), 3, cv::Scalar(255, 255, 0), -1);
          
          cv::imshow("RRT Path Planning", image);

          if (step_mode) {
            planningContinued = false;
          }
          
          if (!isInObstacle(newPoint) && !checkLineCollision(nearest->position, newPoint)) {
              auto newNode = std::make_shared<TreeNode>(newPoint, nearest);
              tree.push_back(newNode);

              cv::circle(image, cv::Point(newPoint.x, newPoint.y), 4, cv::Scalar(255, 255, 0), -1);
              cv::line(image, 
                        cv::Point(newPoint.x, newPoint.y),
                        cv::Point(nearest->position.x, nearest->position.y),
                        cv::Scalar(255, 0, 255), 2);
              cv::imshow("RRT Path Planning", image);

              if (reachedGoal(newNode)) {
                  std::cout << "Goal reached after " << iter << " iterations!" << std::endl;
                  return buildPath(newNode);
              }
          }
        }
        
        std::cout << "Failed to reach goal after " << maxIterations << " iterations." << std::endl;
        // 返回最近的路径
        std::shared_ptr<TreeNode> nearestToGoal = tree[0];
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
    std::vector<Point> buildPath(std::shared_ptr<TreeNode> node) {
        std::vector<Point> path;
        std::shared_ptr<TreeNode> current = node;
        
        while (current != nullptr) {
            path.push_back(current->position);
            current = current->parent;
        }
        
        // 反转路径，使其从起点到终点
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    // 获取树的所有节点
    const std::vector<std::shared_ptr<TreeNode>>& getTree() const {
        return tree;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 初始化窗口
    const int width = 800;
    const int height = 600;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    // 清空图像
    image = cv::Scalar(0, 0, 0);
    
    // 创建窗口
    // 使窗口可调整大小
    cv::namedWindow("RRT Path Planning", cv::WINDOW_NORMAL);
    // 调整窗口大小
    cv::resizeWindow("RRT Path Planning", width, height);
    
    // 定义起点和目标点
    Point start(50, height/2);
    Point goal(width - 50, height/2);
    
    // 定义障碍物
    std::vector<cv::Rect> obstacles {
      cv::Rect(300, 200, 200, 50),
      cv::Rect(450, 300, 50, 150),
      cv::Rect(200, 400, 150, 50)
    };

    // 初始化RRT规划器
    RRTPlanner planner(width, height, start, goal);

    // 添加障碍物
    for (auto& obs : obstacles) {
        planner.addObstacle(obs);
    }
    
    // 绘制起点和终点
    cv::circle(image, cv::Point(start.x, start.y), 8, cv::Scalar(0, 255, 0), -1);
    cv::circle(image, cv::Point(goal.x, goal.y), 8, cv::Scalar(255, 0, 0), -1);
    
    // 绘制障碍物
    for (auto& obs : obstacles) {
        cv::rectangle(image, obs, cv::Scalar(0, 0, 64), -1);
    }
    
    // 显示状态文本
    std::string status = "Press SPACE to start/pause planning, ESC to exit";
    cv::putText(image, status, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // 路径规划标志
    bool planningStarted = false;
    bool planningFinished = false;
    std::vector<Point> path;
    
    // 主循环
    while (rclcpp::ok()) {
        // 执行规划
        if (planningStarted && !planningFinished) {
          path = planner.plan(image);
          planningFinished = true;
          // 绘制路径
          if (!path.empty()) {
            for (size_t i = 0; i < path.size() - 1; ++i) {
              cv::line(image, 
                      cv::Point(path[i].x, path[i].y),
                      cv::Point(path[i+1].x, path[i+1].y),
                      cv::Scalar(0, 255, 255), 2);
              cv::imshow("RRT Path Planning", image);
              cv::waitKey(10);
            }
          }
          std::cout << "Planning complete!" << std::endl;
        }
        
        // 显示图像
        cv::imshow("RRT Path Planning", image);
        
        // 处理按键事件
        int key = cv::waitKey(10);
        if (key == 27) {  // ESC键退出
            rclcpp::shutdown();
        } else if (key == 32 && !planningStarted) {  // 空格键开始规划
            planningStarted = true;
        }
    }

    rclcpp::shutdown();
    return 0;
}    