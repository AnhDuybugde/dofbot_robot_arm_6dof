#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node=std::make_shared<rclcpp::Node>(
        "dofbot_moveit_smoke",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spinner([&](){ exec.spin(); });

    std::this_thread::sleep_for(2s);

    MoveGroupInterface arm(node,"arm_group");
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(10);
    arm.setMaxVelocityScalingFactor(0.20);
    arm.setMaxAccelerationScalingFactor(0.20);

    RCLCPP_INFO(node->get_logger(),"Planning frame: %s",arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(),"EEF: %s",arm.getEndEffectorLink().c_str());

    auto run=[&](const std::string& name)
    {
        arm.setStartStateToCurrentState();

        if(!arm.setNamedTarget(name)){
            RCLCPP_ERROR(node->get_logger(),"Target '%s' not found",name.c_str());
            return false;
        }

        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(node->get_logger(),"PLAN -> %s",name.c_str());

        if(arm.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_ERROR(node->get_logger(),"PLAN FAILED -> %s",name.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(),"EXECUTE -> %s",name.c_str());

        if(arm.execute(plan)!=moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_ERROR(node->get_logger(),"EXECUTE FAILED -> %s",name.c_str());
            return false;
        }

        std::this_thread::sleep_for(1s);
        return true;
    };

    bool ok=run("up") && run("init");

    if(ok)
        RCLCPP_INFO(node->get_logger(),"[PASS] MoveIt plan + execute OK");
    else
        RCLCPP_ERROR(node->get_logger(),"[FAIL] MoveIt smoke test");

    rclcpp::shutdown();
    spinner.join();
    return ok ? 0 : 1;
}
