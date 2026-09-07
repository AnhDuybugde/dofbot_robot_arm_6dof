#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

struct Offset {
    double x,y,z;
    const char* name;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node=std::make_shared<rclcpp::Node>(
        "dofbot_xyz_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spinner([&](){ exec.spin(); });

    std::this_thread::sleep_for(2s);

    MoveGroupInterface arm(node,"arm_group");
    arm.setPlanningTime(8.0);
    arm.setNumPlanningAttempts(15);
    arm.setGoalPositionTolerance(0.01);
    arm.setMaxVelocityScalingFactor(0.15);
    arm.setMaxAccelerationScalingFactor(0.15);

    RCLCPP_INFO(node->get_logger(),"Planning frame = %s",arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(),"Default EEF = %s",arm.getEndEffectorLink().c_str());

    auto named=[&](const std::string& name){
        arm.setStartStateToCurrentState();
        arm.clearPoseTargets();

        if(!arm.setNamedTarget(name)) return false;

        MoveGroupInterface::Plan plan;
        if(arm.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS) return false;
        return arm.execute(plan)==moveit::core::MoveItErrorCode::SUCCESS;
    };

    RCLCPP_INFO(node->get_logger(),"MOVE -> UP");

    if(!named("up")){
        RCLCPP_ERROR(node->get_logger(),"Khong the move den UP");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    std::this_thread::sleep_for(1s);

    auto start=arm.getCurrentPose("arm5_Link").pose;
    double x=start.position.x;
    double y=start.position.y;
    double z=start.position.z;

    RCLCPP_INFO(node->get_logger(),
        "ARM5 START = %.3f %.3f %.3f",x,y,z);

    std::vector<Offset> tests={
        {0.00,0.00,-0.02,"Z -2cm"},
        {0.02,0.00,0.00,"X +2cm"},
        {-0.02,0.00,0.00,"X -2cm"},
        {0.00,0.02,0.00,"Y +2cm"},
        {0.00,-0.02,0.00,"Y -2cm"}
    };

    bool ok=false;

    for(const auto& d:tests){
        arm.setStartStateToCurrentState();
        arm.clearPoseTargets();

        double tx=x+d.x;
        double ty=y+d.y;
        double tz=z+d.z;

        RCLCPP_INFO(node->get_logger(),
            "TRY %s -> %.3f %.3f %.3f",
            d.name,tx,ty,tz);

        arm.setPositionTarget(tx,ty,tz,"arm5_Link");

        MoveGroupInterface::Plan plan;

        if(arm.plan(plan)==moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO(node->get_logger(),"PLAN OK -> %s",d.name);

            if(arm.execute(plan)==moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(node->get_logger(),"EXECUTE OK -> %s",d.name);
                ok=true;
                break;
            }
        }

        RCLCPP_WARN(node->get_logger(),"FAILED -> %s",d.name);
    }

    RCLCPP_INFO(node->get_logger(),"RETURN -> UP");
    named("up");

    if(ok)
        RCLCPP_INFO(node->get_logger(),"[PASS] POSITION IK/PLANNING OK");
    else
        RCLCPP_ERROR(node->get_logger(),"[FAIL] Tat ca position target deu fail");

    rclcpp::shutdown();
    spinner.join();
    return ok ? 0 : 1;
}
