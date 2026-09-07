
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

class SetTargetPosition : public rclcpp::Node
{
public:
  SetTargetPosition()
    : Node("set_target_position")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SetTargetPosition.");
    init_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), 
      std::bind(&SetTargetPosition::initialize, this)
    );
  }

  void initialize()
  {
    init_timer_->cancel();

    // 初始化MoveGroupInterface
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_interface_->setNumPlanningAttempts(10);   // 最大规划尝试次数
    move_group_interface_->setPlanningTime(5.0);        // 每次规划超时时间
    move_group_interface_->setGoalTolerance(0.01);       

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    //：移动到初始位姿（up）
    RCLCPP_INFO(this->get_logger(), "Moving to 'up' pose...");
    move_group_interface_->setNamedTarget("up");
    bool init_success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (init_success)
    {
      RCLCPP_INFO(this->get_logger(), "Init arm succeeded. Executing...");
      move_group_interface_->execute(my_plan);
      // 等待运动完成（2秒）
      rclcpp::sleep_for(std::chrono::seconds(2));
     }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Init arm failed! Exiting...");
      return; // 初始化失败，直接退出
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x =4.279480708646588e-05;
    target_pose.orientation.y = 7.232956704683602e-05;
    target_pose.orientation.z = -2.501120798115153e-05;
    target_pose.orientation.w = 1.0;
    // 目标位置
    target_pose.position.x =-0.12020579725503922;
    target_pose.position.y = 0.07182849198579788;
    target_pose.position.z =0.275874525308609;

    RCLCPP_INFO(this->get_logger(), "Setting target pose...");
    move_group_interface_->setPoseTarget(target_pose);

    bool pose_success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (pose_success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded, moving the arm.");
      move_group_interface_->execute(my_plan);
      rclcpp::sleep_for(std::chrono::seconds(5)); // 等待运动完成


    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }

    // 执行完成后关闭节点
    rclcpp::shutdown();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr init_timer_; // 延迟初始化定时器
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetTargetPosition>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
 
  std::thread executor_thread([&executor]() {
    executor.spin();
  });
  executor_thread.join();
  return 0;
}