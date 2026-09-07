#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

static constexpr double PI =
    3.14159265358979323846;

static constexpr const char* TIP =
    "Gripping_point_Link";

static constexpr const char* TABLE_ID =
    "tea_table";

static constexpr const char* CUP_ID =
    "tea_cup";

static constexpr const char* POT_ID =
    "tea_teapot";

struct MeshData
{
    shape_msgs::msg::Mesh mesh;

    double min_x=std::numeric_limits<double>::infinity();
    double min_y=std::numeric_limits<double>::infinity();
    double min_z=std::numeric_limits<double>::infinity();

    double max_x=-std::numeric_limits<double>::infinity();
    double max_y=-std::numeric_limits<double>::infinity();
    double max_z=-std::numeric_limits<double>::infinity();
};

static geometry_msgs::msg::Pose poseXYZYaw(
    double x,
    double y,
    double z,
    double yaw_deg)
{
    geometry_msgs::msg::Pose p;

    p.position.x=x;
    p.position.y=y;
    p.position.z=z;

    tf2::Quaternion q;

    q.setRPY(
        0.0,
        0.0,
        yaw_deg*PI/180.0
    );

    p.orientation=tf2::toMsg(q);

    return p;
}

static tf2::Transform poseToTf(
    const geometry_msgs::msg::Pose& p)
{
    tf2::Transform t;

    t.setOrigin(
        tf2::Vector3(
            p.position.x,
            p.position.y,
            p.position.z
        )
    );

    tf2::Quaternion q;

    tf2::fromMsg(
        p.orientation,
        q
    );

    t.setRotation(q);

    return t;
}

static geometry_msgs::msg::Pose tfToPose(
    const tf2::Transform& t)
{
    geometry_msgs::msg::Pose p;

    p.position.x=t.getOrigin().x();
    p.position.y=t.getOrigin().y();
    p.position.z=t.getOrigin().z();

    p.orientation=tf2::toMsg(
        t.getRotation()
    );

    return p;
}

static void updateBounds(
    MeshData& out,
    double x,
    double y,
    double z)
{
    out.min_x=std::min(out.min_x,x);
    out.min_y=std::min(out.min_y,y);
    out.min_z=std::min(out.min_z,z);

    out.max_x=std::max(out.max_x,x);
    out.max_y=std::max(out.max_y,y);
    out.max_z=std::max(out.max_z,z);
}

static void addTriangle(
    MeshData& out,
    double x1,
    double y1,
    double z1,
    double x2,
    double y2,
    double z2,
    double x3,
    double y3,
    double z3,
    double scale)
{
    uint32_t base=
        static_cast<uint32_t>(
            out.mesh.vertices.size()
        );

    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    geometry_msgs::msg::Point p3;

    p1.x=x1*scale;
    p1.y=y1*scale;
    p1.z=z1*scale;

    p2.x=x2*scale;
    p2.y=y2*scale;
    p2.z=z2*scale;

    p3.x=x3*scale;
    p3.y=y3*scale;
    p3.z=z3*scale;

    out.mesh.vertices.push_back(p1);
    out.mesh.vertices.push_back(p2);
    out.mesh.vertices.push_back(p3);

    shape_msgs::msg::MeshTriangle tri;

    tri.vertex_indices[0]=base;
    tri.vertex_indices[1]=base+1;
    tri.vertex_indices[2]=base+2;

    out.mesh.triangles.push_back(tri);

    updateBounds(out,p1.x,p1.y,p1.z);
    updateBounds(out,p2.x,p2.y,p2.z);
    updateBounds(out,p3.x,p3.y,p3.z);
}

static MeshData loadSTL(
    const std::string& path,
    double scale)
{
    std::ifstream f(
        path,
        std::ios::binary
    );

    if(!f)
        throw std::runtime_error(
            "Cannot open STL: "+path
        );

    std::vector<unsigned char> data(
        (std::istreambuf_iterator<char>(f)),
        std::istreambuf_iterator<char>()
    );

    MeshData out;

    if(data.size()>=84){
        uint32_t count=0;

        std::memcpy(
            &count,
            data.data()+80,
            sizeof(uint32_t)
        );

        size_t expected=
            84+
            static_cast<size_t>(count)*50;

        if(expected==data.size()){
            for(uint32_t i=0;i<count;i++){
                size_t off=
                    84+
                    static_cast<size_t>(i)*50+
                    12;

                float v[9];

                std::memcpy(
                    v,
                    data.data()+off,
                    sizeof(v)
                );

                addTriangle(
                    out,
                    v[0],v[1],v[2],
                    v[3],v[4],v[5],
                    v[6],v[7],v[8],
                    scale
                );
            }

            if(out.mesh.triangles.empty())
                throw std::runtime_error(
                    "Empty binary STL: "+path
                );

            return out;
        }
    }

    std::string text(
        data.begin(),
        data.end()
    );

    std::regex re(
        R"(vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+))"
    );

    std::vector<double> values;

    for(
        std::sregex_iterator it(
            text.begin(),
            text.end(),
            re
        ),
        end;
        it!=end;
        ++it)
    {
        values.push_back(
            std::stod((*it)[1].str())
        );

        values.push_back(
            std::stod((*it)[2].str())
        );

        values.push_back(
            std::stod((*it)[3].str())
        );
    }

    if(values.size()%9!=0)
        throw std::runtime_error(
            "Invalid ASCII STL: "+path
        );

    for(size_t i=0;i<values.size();i+=9){
        addTriangle(
            out,
            values[i+0],
            values[i+1],
            values[i+2],
            values[i+3],
            values[i+4],
            values[i+5],
            values[i+6],
            values[i+7],
            values[i+8],
            scale
        );
    }

    if(out.mesh.triangles.empty())
        throw std::runtime_error(
            "No triangles in STL: "+path
        );

    return out;
}

class TeaTask
{
public:
    explicit TeaTask(
        const rclcpp::Node::SharedPtr& node)
        :
        node_(node),
        arm_(node_,"arm_group"),
        grip_(node_,"grip_group")
    {
    }

    bool init()
    {
        planning_time_=
            param("planning_time",8.0);

        planning_attempts_=
            static_cast<int>(
                paramInt(
                    "planning_attempts",
                    15
                )
            );

        velocity_scaling_=
            param(
                "velocity_scaling",
                0.15
            );

        acceleration_scaling_=
            param(
                "acceleration_scaling",
                0.10
            );

        position_tolerance_=
            param(
                "position_tolerance",
                0.008
            );

        joint_tolerance_=
            param(
                "joint_tolerance_rad",
                0.03
            );

        carry_orientation_tolerance_=
            param(
                "carry_orientation_tolerance_rad",
                0.55
            );

        arm_.setPlanningTime(
            planning_time_
        );

        arm_.setNumPlanningAttempts(
            planning_attempts_
        );

        arm_.setMaxVelocityScalingFactor(
            velocity_scaling_
        );

        arm_.setMaxAccelerationScalingFactor(
            acceleration_scaling_
        );

        arm_.setGoalPositionTolerance(
            position_tolerance_
        );

        grip_.setPlanningTime(
            4.0
        );

        grip_.setNumPlanningAttempts(
            10
        );

        grip_.setMaxVelocityScalingFactor(
            velocity_scaling_
        );

        grip_.setMaxAccelerationScalingFactor(
            acceleration_scaling_
        );

        if(!arm_.startStateMonitor(3.0)){
            fail(
                "arm current state unavailable"
            );

            return false;
        }

        if(!grip_.startStateMonitor(3.0)){
            fail(
                "gripper current state unavailable"
            );

            return false;
        }

        if(!loadGeometry())
            return false;

        if(!computeTargets())
            return false;

        if(!setupScene())
            return false;

        // RViz' Tea Scene display subscribes to this visual representation.
        // It is independent of the collision meshes used by MoveIt planning.
        rclcpp::QoS scene_qos(1);
        scene_qos.reliable().transient_local();
        scene_pub_=node_->create_publisher<
            visualization_msgs::msg::MarkerArray
        >("/tea_scene",scene_qos);
        publishVisualScene();
        scene_timer_=node_->create_wall_timer(
            500ms,[this](){ publishVisualScene(); }
        );

        service_group_=
            node_->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant
            );

        next_service_=
            node_->create_service<
                std_srvs::srv::Trigger
            >(
                "/tea/next",
                std::bind(
                    &TeaTask::onNext,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                ),
                rmw_qos_profile_services_default,
                service_group_
            );

        status_service_=
            node_->create_service<
                std_srvs::srv::Trigger
            >(
                "/tea/status",
                std::bind(
                    &TeaTask::onStatus,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                ),
                rmw_qos_profile_services_default,
                service_group_
            );

        abort_service_=
            node_->create_service<
                std_srvs::srv::Trigger
            >(
                "/tea/abort",
                std::bind(
                    &TeaTask::onAbort,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                ),
                rmw_qos_profile_services_default,
                service_group_
            );

        RCLCPP_INFO(
            node_->get_logger(),
            "[READY] scene=table+cup+teapot"
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "[READY] next=%s",
            stepName(step_).c_str()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "[READY] task does not move until /tea/next is called"
        );

        return true;
    }

private:
    void publishVisualScene()
    {
        if(!scene_pub_) return;
        visualization_msgs::msg::MarkerArray a;
        auto add=[&](int id,const std::string& ns,int type,
                     const std::string& frame,
                     const geometry_msgs::msg::Pose& pose,
                     double sx,double sy,double sz,
                     const std::string& mesh=""){
            visualization_msgs::msg::Marker m;
            m.header.frame_id=frame;
            m.header.stamp=node_->now();
            m.ns=ns; m.id=id; m.type=type;
            m.action=visualization_msgs::msg::Marker::ADD;
            m.pose=pose; m.scale.x=sx; m.scale.y=sy; m.scale.z=sz;
            m.color.r=0.75; m.color.g=0.55; m.color.b=0.25; m.color.a=1.0;
            m.mesh_resource=mesh;
            m.mesh_use_embedded_materials=false;
            a.markers.push_back(m);
        };
        add(1,"tea_table",visualization_msgs::msg::Marker::CUBE,
            "base_link",poseXYZYaw(table_x_,table_y_,table_z_,0),
            table_sx_,table_sy_,table_sz_);
        add(10,"tea_cup",visualization_msgs::msg::Marker::MESH_RESOURCE,
            "base_link",cup_pose_,cup_scale_,cup_scale_,cup_scale_,
            "package://dofbot_urdf/meshes/objects/TeaCup.stl");
        add(20,"tea_teapot",visualization_msgs::msg::Marker::MESH_RESOURCE,
            pot_attached_ ? TIP : "base_link",
            pot_attached_ ? pot_relative_pose_ : pot_world_pose_,
            pot_scale_,pot_scale_,pot_scale_,
            "package://dofbot_urdf/meshes/objects/WaterSprayingTeapot.stl");
        scene_pub_->publish(a);
    }

    double param(
        const std::string& name,
        double value)
    {
        if(!node_->has_parameter(name))
            node_->declare_parameter<double>(
                name,
                value
            );

        return node_->get_parameter(
            name
        ).as_double();
    }

    int64_t paramInt(
        const std::string& name,
        int64_t value)
    {
        if(!node_->has_parameter(name))
            node_->declare_parameter<int64_t>(
                name,
                value
            );

        return node_->get_parameter(
            name
        ).as_int();
    }

    void fail(
        const std::string& text)
    {
        failed_=true;

        arm_.stop();
        grip_.stop();

        RCLCPP_ERROR(
            node_->get_logger(),
            "[FAIL] %s",
            text.c_str()
        );
    }

    bool loadGeometry()
    {
        try{
            std::string share=
                ament_index_cpp::
                    get_package_share_directory(
                        "dofbot_urdf"
                    );

            std::string cup_path=
                share+
                "/meshes/objects/TeaCup.stl";

            std::string pot_path=
                share+
                "/meshes/objects/WaterSprayingTeapot.stl";

            cup_scale_=
                param(
                    "cup_scale",
                    0.00085
                );

            pot_scale_=
                param(
                    "teapot_scale",
                    0.001
                );

            cal_cup_scale_=
                param(
                    "cal_cup_scale",
                    0.00085
                );

            cup_mesh_=
                loadSTL(
                    cup_path,
                    cup_scale_
                );

            pot_mesh_=
                loadSTL(
                    pot_path,
                    pot_scale_
                );

            cal_cup_mesh_=
                loadSTL(
                    cup_path,
                    cal_cup_scale_
                );

            RCLCPP_INFO(
                node_->get_logger(),
                "[SCENE] STL loaded cup_tri=%zu pot_tri=%zu",
                cup_mesh_.mesh.triangles.size(),
                pot_mesh_.mesh.triangles.size()
            );

            return true;
        }
        catch(const std::exception& e){
            fail(e.what());
            return false;
        }
    }

    bool computeTargets()
    {
        table_x_=param("table_x",0.10);
        table_y_=param("table_y",0.0);
        table_z_=param("table_z",-0.011);

        table_sx_=param(
            "table_size_x",
            0.38
        );

        table_sy_=param(
            "table_size_y",
            0.34
        );

        table_sz_=param(
            "table_size_z",
            0.02
        );

        support_eps_=
            param(
                "support_eps",
                0.001
            );

        double table_top=
            table_z_+
            table_sz_/2.0;

        double cup_x=
            param(
                "cup_x",
                0.095
            );

        double cup_y=
            param(
                "cup_y",
                0.080
            );

        double cup_yaw=
            param(
                "cup_yaw_deg",
                0.0
            );

        double cup_z=
            table_top-
            cup_mesh_.min_z+
            support_eps_;

        cup_pose_=
            poseXYZYaw(
                cup_x,
                cup_y,
                cup_z,
                cup_yaw
            );

        double cup_top=
            cup_z+
            cup_mesh_.max_z;

        double pot_x=
            param(
                "teapot_x",
                0.14
            );

        double pot_y=
            param(
                "teapot_y",
                -0.01
            );

        double pot_yaw=
            param(
                "teapot_yaw_deg",
                90.0
            );

        double pot_z=
            table_top-
            pot_mesh_.min_z+
            support_eps_;

        pot_world_pose_=
            poseXYZYaw(
                pot_x,
                pot_y,
                pot_z,
                pot_yaw
            );

        double hx=
            param(
                "handle_local_x_mm",
                0.0
            )*
            pot_scale_;

        double hy=
            param(
                "handle_local_y_mm",
                32.0
            )*
            pot_scale_;

        double hz=
            param(
                "handle_local_z_mm",
                50.0
            )*
            pot_scale_;

        tf2::Transform Tbp=
            poseToTf(
                pot_world_pose_
            );

        tf2::Vector3 handle=
            Tbp*
            tf2::Vector3(
                hx,
                hy,
                hz
            );

        grasp_=
            handle;

        above_=
            grasp_;

        above_.setZ(
            grasp_.z()+
            param(
                "handle_approach_dz",
                0.025
            )
        );

        double cal_cup_x=
            param(
                "cal_cup_x",
                0.095
            );

        double cal_cup_y=
            param(
                "cal_cup_y",
                0.080
            );

        double cal_grip_x=
            param(
                "cal_grip_x",
                0.110
            );

        double cal_grip_y=
            param(
                "cal_grip_y",
                0.075
            );

        double th=
            std::atan2(
                cal_grip_y,
                cal_grip_x
            );

        double dx=
            cal_cup_x-
            cal_grip_x;

        double dy=
            cal_cup_y-
            cal_grip_y;

        double radial=
            std::cos(th)*dx+
            std::sin(th)*dy;

        double tangential=
            -std::sin(th)*dx+
            std::cos(th)*dy;

        double R=
            std::hypot(
                cup_x,
                cup_y
            );

        if(
            R<=1e-6 ||
            std::abs(tangential)>R)
        {
            fail(
                "invalid cup XY calibration"
            );

            return false;
        }

        double beta=
            std::atan2(
                cup_y,
                cup_x
            );

        double d1=
            std::asin(
                tangential/R
            );

        bool found=false;
        double best_score=
            std::numeric_limits<double>::infinity();

        double target_x=0.0;
        double target_y=0.0;

        for(double delta:
            std::vector<double>{
                d1,
                PI-d1
            })
        {
            double rho=
                R*
                std::cos(delta)-
                radial;

            if(rho<=0.0)
                continue;

            double theta=
                beta-
                delta;

            double gx=
                rho*
                std::cos(theta);

            double gy=
                rho*
                std::sin(theta);

            double diff=
                std::atan2(
                    std::sin(theta-beta),
                    std::cos(theta-beta)
                );

            double score=
                std::abs(diff);

            if(score<best_score){
                found=true;
                best_score=score;
                target_x=gx;
                target_y=gy;
            }
        }

        if(!found){
            fail(
                "no calibrated cup target"
            );

            return false;
        }

        double cal_table_top=
            param(
                "cal_table_top_z",
                -0.001
            );

        double cal_cup_z=
            cal_table_top-
            cal_cup_mesh_.min_z+
            support_eps_;

        double cal_cup_top=
            cal_cup_z+
            cal_cup_mesh_.max_z;

        double cal_grasp_z=
            param(
                "cal_grasp_z",
                0.050
            );

        double cal_lift_z=
            param(
                "cal_lift_z",
                0.090
            );

        double cal_approach_z=
            param(
                "cal_approach_z",
                0.083
            );

        double cal_pour_z=
            param(
                "cal_pour_z",
                0.080
            );

        double pour_top_offset=
            cal_pour_z-
            cal_cup_top;

        double approach_extra=
            cal_approach_z-
            cal_pour_z;

        double carry_margin=
            cal_lift_z-
            cal_approach_z;

        double lift_from_grasp=
            cal_lift_z-
            cal_grasp_z;

        double cup_pour_z=
            cup_top+
            pour_top_offset;

        double cup_approach_z=
            cup_pour_z+
            approach_extra;

        pour_position_=
            tf2::Vector3(
                target_x,
                target_y,
                cup_pour_z
            );

        cup_approach_=
            tf2::Vector3(
                target_x,
                target_y,
                cup_approach_z
            );

        double lift_z=
            std::max(
                grasp_.z()+
                lift_from_grasp,
                cup_approach_z+
                carry_margin
            );

        lift_=
            tf2::Vector3(
                grasp_.x(),
                grasp_.y(),
                lift_z
            );

        pour_delta_deg_=
            param(
                "pour_joint_delta_deg",
                -15.0
            );

        RCLCPP_INFO(
            node_->get_logger(),
            "[TARGET] grasp=[%.3f %.3f %.3f]",
            grasp_.x(),
            grasp_.y(),
            grasp_.z()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "[TARGET] lift=[%.3f %.3f %.3f]",
            lift_.x(),
            lift_.y(),
            lift_.z()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "[TARGET] cup=[%.3f %.3f %.3f]",
            pour_position_.x(),
            pour_position_.y(),
            pour_position_.z()
        );

        return true;
    }

    moveit_msgs::msg::CollisionObject tableObject()
    {
        moveit_msgs::msg::CollisionObject o;

        o.header.frame_id=
            "base_link";

        o.id=TABLE_ID;

        shape_msgs::msg::SolidPrimitive box;

        box.type=
            shape_msgs::msg::
                SolidPrimitive::BOX;

        box.dimensions.resize(3);

        box.dimensions[0]=
            table_sx_;

        box.dimensions[1]=
            table_sy_;

        box.dimensions[2]=
            table_sz_;

        geometry_msgs::msg::Pose p=
            poseXYZYaw(
                table_x_,
                table_y_,
                table_z_,
                0.0
            );

        o.primitives.push_back(box);
        o.primitive_poses.push_back(p);

        o.operation=
            moveit_msgs::msg::
                CollisionObject::ADD;

        return o;
    }

    moveit_msgs::msg::CollisionObject meshObject(
        const std::string& id,
        const MeshData& mesh,
        const geometry_msgs::msg::Pose& pose)
    {
        moveit_msgs::msg::CollisionObject o;

        o.header.frame_id=
            "base_link";

        o.id=id;

        o.meshes.push_back(
            mesh.mesh
        );

        o.mesh_poses.push_back(
            pose
        );

        o.operation=
            moveit_msgs::msg::
                CollisionObject::ADD;

        return o;
    }

    bool setupScene()
    {
        planning_scene_.removeCollisionObjects(
            {
                TABLE_ID,
                CUP_ID,
                POT_ID
            }
        );

        std::vector<
            moveit_msgs::msg::CollisionObject
        > objects;

        objects.push_back(
            tableObject()
        );

        objects.push_back(
            meshObject(
                CUP_ID,
                cup_mesh_,
                cup_pose_
            )
        );

        objects.push_back(
            meshObject(
                POT_ID,
                pot_mesh_,
                pot_world_pose_
            )
        );

        if(
            !planning_scene_.
                applyCollisionObjects(
                    objects
                )
        ){
            fail(
                "PlanningScene apply failed"
            );

            return false;
        }

        std::this_thread::sleep_for(
            300ms
        );

        auto known=
            planning_scene_.getObjects(
                {
                    TABLE_ID,
                    CUP_ID,
                    POT_ID
                }
            );

        if(known.size()!=3){
            fail(
                "PlanningScene object sync failed"
            );

            return false;
        }

        pot_world_present_=true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[SCENE] world=table,cup,teapot"
        );

        return true;
    }

    bool removeWorldPot()
    {
        if(!pot_world_present_)
            return true;

        planning_scene_.
            removeCollisionObjects(
                {POT_ID}
            );

        std::this_thread::sleep_for(
            200ms
        );

        auto known=
            planning_scene_.
                getObjects(
                    {POT_ID}
                );

        if(!known.empty()){
            fail(
                "cannot open teapot grasp contact window"
            );

            return false;
        }

        pot_world_present_=false;

        RCLCPP_INFO(
            node_->get_logger(),
            "[SCENE] teapot grasp contact allowed"
        );

        return true;
    }

    bool attachPot()
    {
        auto tool=
            arm_.getCurrentPose(
                TIP
            ).pose;

        tf2::Transform Tbg=
            poseToTf(tool);

        tf2::Transform Tbp=
            poseToTf(
                pot_world_pose_
            );

        tf2::Transform Tgp=
            Tbg.inverse()*
            Tbp;

        pot_relative_pose_=
            tfToPose(
                Tgp
            );

        moveit_msgs::msg::
            AttachedCollisionObject a;

        a.link_name=TIP;

        a.object.header.frame_id=
            TIP;

        a.object.id=
            POT_ID;

        a.object.meshes.push_back(
            pot_mesh_.mesh
        );

        a.object.mesh_poses.push_back(
            pot_relative_pose_
        );

        a.object.operation=
            moveit_msgs::msg::
                CollisionObject::ADD;

        a.touch_links=
            grip_.getLinkNames();

        a.touch_links.push_back(
            "arm5_Link"
        );

        a.touch_links.push_back(
            TIP
        );

        if(
            !planning_scene_.
                applyAttachedCollisionObject(
                    a
                )
        ){
            fail(
                "attach teapot collision failed"
            );

            return false;
        }

        pot_attached_=true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[SCENE] teapot attached collision active"
        );

        return true;
    }

    bool detachPot()
    {
        if(!pot_attached_)
            return true;

        auto tool=
            arm_.getCurrentPose(
                TIP
            ).pose;

        tf2::Transform Tbg=
            poseToTf(
                tool
            );

        tf2::Transform Tgp=
            poseToTf(
                pot_relative_pose_
            );

        tf2::Transform Tbp=
            Tbg*
            Tgp;

        pot_world_pose_=
            tfToPose(
                Tbp
            );

        moveit_msgs::msg::
            AttachedCollisionObject a;

        a.link_name=TIP;

        a.object.id=POT_ID;

        a.object.operation=
            moveit_msgs::msg::
                CollisionObject::REMOVE;

        if(
            !planning_scene_.
                applyAttachedCollisionObject(
                    a
                )
        ){
            fail(
                "detach teapot collision failed"
            );

            return false;
        }

        pot_attached_=false;

        RCLCPP_INFO(
            node_->get_logger(),
            "[SCENE] teapot detached, release contact allowed"
        );

        return true;
    }

    bool readdWorldPot()
    {
        auto object=
            meshObject(
                POT_ID,
                pot_mesh_,
                pot_world_pose_
            );

        if(
            !planning_scene_.
                applyCollisionObject(
                    object
                )
        ){
            fail(
                "re-add teapot collision failed"
            );

            return false;
        }

        pot_world_present_=true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[SCENE] world=table,cup,teapot"
        );

        return true;
    }

    moveit_msgs::msg::Constraints
    orientationConstraint(
        const geometry_msgs::msg::Quaternion& q)
    {
        moveit_msgs::msg::Constraints c;

        moveit_msgs::msg::
            OrientationConstraint o;

        o.header.frame_id=
            "base_link";

        o.link_name=
            TIP;

        o.orientation=q;

        o.absolute_x_axis_tolerance=
            carry_orientation_tolerance_;

        o.absolute_y_axis_tolerance=
            carry_orientation_tolerance_;

        o.absolute_z_axis_tolerance=
            PI;

        o.weight=1.0;

        c.orientation_constraints.
            push_back(o);

        return c;
    }

    bool verifyTool(
        const tf2::Vector3& target,
        const std::string& label)
    {
        auto pose=
            arm_.getCurrentPose(
                TIP
            ).pose;

        tf2::Vector3 actual(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );

        double error=
            (target-actual).length();

        if(error>
           position_tolerance_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[VERIFY] %s FAIL error=%.1fmm",
                label.c_str(),
                error*1000.0
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[VERIFY] %s PASS error=%.1fmm",
            label.c_str(),
            error*1000.0
        );

        return true;
    }

    bool moveTool(
        const tf2::Vector3& target,
        const std::string& label,
        bool keep_orientation)
    {
        if(
            !arm_.getCurrentState(
                2.0
            )
        ){
            fail(
                label+
                ": current state unavailable"
            );

            return false;
        }

        arm_.setStartStateToCurrentState();
        arm_.clearPoseTargets();
        arm_.clearPathConstraints();

        if(keep_orientation){
            arm_.setPathConstraints(
                orientationConstraint(
                    carry_orientation_
                )
            );
        }

        arm_.setPositionTarget(
            target.x(),
            target.y(),
            target.z(),
            TIP
        );

        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s",
            label.c_str()
        );

        auto result=
            arm_.plan(plan);

        arm_.clearPoseTargets();
        arm_.clearPathConstraints();

        if(
            result!=
            moveit::core::
                MoveItErrorCode::SUCCESS ||
            plan.trajectory_.
                joint_trajectory.
                points.empty()
        ){
            fail(
                label+
                ": plan failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s PASS points=%zu",
            label.c_str(),
            plan.trajectory_.
                joint_trajectory.
                points.size()
        );

        auto execute_result=
            arm_.execute(plan);

        if(
            execute_result!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": execute failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[EXEC] %s PASS",
            label.c_str()
        );

        std::this_thread::sleep_for(
            150ms
        );

        if(!verifyTool(
            target,
            label
        )){
            fail(
                label+
                ": verify failed"
            );

            return false;
        }

        return true;
    }

    bool verifyNamed(
        MoveGroupInterface& group,
        const std::map<
            std::string,
            double
        >& target,
        const std::string& label)
    {
        auto names=
            group.getJointNames();

        auto actual=
            group.getCurrentJointValues();

        if(
            names.size()!=
            actual.size()
        ){
            fail(
                label+
                ": joint state size mismatch"
            );

            return false;
        }

        double max_error=0.0;

        for(size_t i=0;i<names.size();i++){
            auto it=
                target.find(
                    names[i]
                );

            if(it==target.end())
                continue;

            max_error=
                std::max(
                    max_error,
                    std::abs(
                        actual[i]-
                        it->second
                    )
                );
        }

        if(max_error>
           joint_tolerance_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[VERIFY] %s FAIL error=%.2fdeg",
                label.c_str(),
                max_error*180.0/PI
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[VERIFY] %s PASS error=%.2fdeg",
            label.c_str(),
            max_error*180.0/PI
        );

        return true;
    }

    bool namedArm(
        const std::string& target_name,
        const std::string& label)
    {
        arm_.setStartStateToCurrentState();
        arm_.clearPoseTargets();
        arm_.clearPathConstraints();

        if(
            !arm_.setNamedTarget(
                target_name
            )
        ){
            fail(
                label+
                ": named target missing"
            );

            return false;
        }

        auto values=
            arm_.getNamedTargetValues(
                target_name
            );

        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s",
            label.c_str()
        );

        if(
            arm_.plan(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": plan failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s PASS",
            label.c_str()
        );

        if(
            arm_.execute(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": execute failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[EXEC] %s PASS",
            label.c_str()
        );

        if(
            !verifyNamed(
                arm_,
                values,
                label
            )
        ){
            fail(
                label+
                ": verify failed"
            );

            return false;
        }

        return true;
    }

    bool namedGrip(
        const std::string& target_name,
        const std::string& label)
    {
        grip_.setStartStateToCurrentState();
        grip_.clearPoseTargets();

        if(
            !grip_.setNamedTarget(
                target_name
            )
        ){
            fail(
                label+
                ": named target missing"
            );

            return false;
        }

        auto values=
            grip_.getNamedTargetValues(
                target_name
            );

        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s",
            label.c_str()
        );

        if(
            grip_.plan(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": plan failed"
            );

            return false;
        }

        if(
            grip_.execute(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": execute failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[EXEC] %s PASS",
            label.c_str()
        );

        if(
            !verifyNamed(
                grip_,
                values,
                label
            )
        ){
            fail(
                label+
                ": verify failed"
            );

            return false;
        }

        return true;
    }

    bool moveJoints(
        const std::vector<double>& target,
        const std::string& label)
    {
        arm_.setStartStateToCurrentState();
        arm_.clearPoseTargets();
        arm_.clearPathConstraints();

        if(
            !arm_.setJointValueTarget(
                target
            )
        ){
            fail(
                label+
                ": invalid joint target"
            );

            return false;
        }

        MoveGroupInterface::Plan plan;

        RCLCPP_INFO(
            node_->get_logger(),
            "[PLAN] %s",
            label.c_str()
        );

        if(
            arm_.plan(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": plan failed"
            );

            return false;
        }

        if(
            arm_.execute(plan)!=
            moveit::core::
                MoveItErrorCode::SUCCESS
        ){
            fail(
                label+
                ": execute failed"
            );

            return false;
        }

        auto actual=
            arm_.getCurrentJointValues();

        if(
            actual.size()!=
            target.size()
        ){
            fail(
                label+
                ": joint size mismatch"
            );

            return false;
        }

        double error=0.0;

        for(size_t i=0;i<target.size();i++)
            error=
                std::max(
                    error,
                    std::abs(
                        actual[i]-
                        target[i]
                    )
                );

        if(error>
           joint_tolerance_)
        {
            fail(
                label+
                ": joint verify failed"
            );

            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[EXEC] %s PASS",
            label.c_str()
        );

        RCLCPP_INFO(
            node_->get_logger(),
            "[VERIFY] %s PASS error=%.2fdeg",
            label.c_str(),
            error*180.0/PI
        );

        return true;
    }

    bool doPour()
    {
        upright_joints_=
            arm_.getCurrentJointValues();

        auto names=
            arm_.getJointNames();

        auto it=
            std::find(
                names.begin(),
                names.end(),
                "arm4_Joint"
            );

        if(it==names.end()){
            fail(
                "POUR: arm4_Joint missing"
            );

            return false;
        }

        size_t index=
            static_cast<size_t>(
                std::distance(
                    names.begin(),
                    it
                )
            );

        double delta=
            pour_delta_deg_*
            PI/180.0;

        auto target=
            upright_joints_;

        target[index]+=
            delta;

        const auto& bounds=
            arm_.getRobotModel()->
                getVariableBounds(
                    "arm4_Joint"
                );

        if(
            bounds.position_bounded_ &&
            (
                target[index]<
                bounds.min_position_ ||
                target[index]>
                bounds.max_position_
            )
        ){
            fail(
                "POUR: configured direction exceeds robot joint limit"
            );

            return false;
        }

        return moveJoints(
            target,
            "POUR"
        );
    }

    std::string stepName(
        size_t i) const
    {
        static const std::vector<
            std::string
        > names={
            "HOME",
            "OPEN GRIPPER",
            "APPROACH HANDLE",
            "LOWER TO HANDLE",
            "CLOSE + ATTACH",
            "LIFT",
            "TRANSFER",
            "LOWER TO POUR",
            "POUR",
            "UPRIGHT",
            "LIFT AWAY",
            "RETURN HIGH",
            "LOWER TO PLACE",
            "RELEASE",
            "RETRACT",
            "HOME FINAL"
        };

        if(i>=names.size())
            return "DONE";

        return names[i];
    }

    bool executeStep(
        size_t i)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "[STEP %02zu/16] %s",
            i+1,
            stepName(i).c_str()
        );

        switch(i){
        case 0:
            return namedArm(
                "up",
                "HOME"
            );

        case 1:
            return namedGrip(
                "open",
                "OPEN GRIPPER"
            );

        case 2:
            return moveTool(
                above_,
                "APPROACH HANDLE",
                false
            );

        case 3:
            if(!removeWorldPot())
                return false;

            if(
                !moveTool(
                    grasp_,
                    "LOWER TO HANDLE",
                    false
                )
            ){
                readdWorldPot();
                return false;
            }

            return true;

        case 4:
            if(
                !namedGrip(
                    "close",
                    "CLOSE GRIPPER"
                )
            )
                return false;

            carry_orientation_=
                arm_.getCurrentPose(
                    TIP
                ).pose.orientation;

            return attachPot();

        case 5:
            return moveTool(
                lift_,
                "LIFT",
                true
            );

        case 6:
            return moveTool(
                cup_approach_,
                "TRANSFER",
                true
            );

        case 7:
            return moveTool(
                pour_position_,
                "LOWER TO POUR",
                true
            );

        case 8:
            return doPour();

        case 9:
            return moveJoints(
                upright_joints_,
                "UPRIGHT"
            );

        case 10:
            return moveTool(
                cup_approach_,
                "LIFT AWAY",
                true
            );

        case 11:
            return moveTool(
                lift_,
                "RETURN HIGH",
                true
            );

        case 12:
            return moveTool(
                grasp_,
                "LOWER TO PLACE",
                true
            );

        case 13:
            if(
                !namedGrip(
                    "open",
                    "RELEASE"
                )
            )
                return false;

            return detachPot();

        case 14:
            if(
                !moveTool(
                    above_,
                    "RETRACT",
                    false
                )
            )
                return false;

            return readdWorldPot();

        case 15:
            return namedArm(
                "up",
                "HOME FINAL"
            );

        default:
            return true;
        }
    }

    void onNext(
        const std::shared_ptr<
            std_srvs::srv::Trigger::Request>,
        std::shared_ptr<
            std_srvs::srv::Trigger::Response
        > response)
    {
        if(busy_.exchange(true)){
            response->success=false;
            response->message="BUSY";
            return;
        }

        struct Guard
        {
            std::atomic<bool>& value;

            ~Guard()
            {
                value=false;
            }
        } guard{busy_};

        if(aborted_){
            response->success=false;
            response->message="ABORTED";
            return;
        }

        if(failed_){
            response->success=false;
            response->message="FAILED - restart task after fixing the cause";
            return;
        }

        if(step_>=16){
            response->success=true;
            response->message="DONE";
            return;
        }

        std::string current=
            stepName(step_);

        if(!executeStep(step_)){
            response->success=false;
            response->message=
                "FAIL at "+current;

            return;
        }

        step_++;

        if(step_>=16){
            RCLCPP_INFO(
                node_->get_logger(),
                "[DONE] MOVEIT TEA PIPELINE PASS"
            );

            response->success=true;
            response->message="PASS - DONE";
            return;
        }

        response->success=true;
        response->message=
            "PASS - next: "+
            stepName(step_);
    }

    void onStatus(
        const std::shared_ptr<
            std_srvs::srv::Trigger::Request>,
        std::shared_ptr<
            std_srvs::srv::Trigger::Response
        > response)
    {
        if(aborted_){
            response->success=false;
            response->message="ABORTED";
            return;
        }

        if(failed_){
            response->success=false;
            response->message=
                "FAILED at/after step "+
                std::to_string(
                    step_+1
                );

            return;
        }

        if(step_>=16){
            response->success=true;
            response->message="DONE";
            return;
        }

        response->success=true;

        response->message=
            "next step "+
            std::to_string(
                step_+1
            )+
            "/16: "+
            stepName(step_);
    }

    void onAbort(
        const std::shared_ptr<
            std_srvs::srv::Trigger::Request>,
        std::shared_ptr<
            std_srvs::srv::Trigger::Response
        > response)
    {
        aborted_=true;

        arm_.stop();
        grip_.stop();

        RCLCPP_WARN(
            node_->get_logger(),
            "[FAIL] operator abort"
        );

        response->success=true;
        response->message="ABORTED";
    }

private:
    rclcpp::Node::SharedPtr node_;

    MoveGroupInterface arm_;
    MoveGroupInterface grip_;

    moveit::planning_interface::
        PlanningSceneInterface
            planning_scene_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_pub_;
    rclcpp::TimerBase::SharedPtr scene_timer_;

    rclcpp::CallbackGroup::SharedPtr
        service_group_;

    rclcpp::Service<
        std_srvs::srv::Trigger
    >::SharedPtr next_service_;

    rclcpp::Service<
        std_srvs::srv::Trigger
    >::SharedPtr status_service_;

    rclcpp::Service<
        std_srvs::srv::Trigger
    >::SharedPtr abort_service_;

    MeshData cup_mesh_;
    MeshData pot_mesh_;
    MeshData cal_cup_mesh_;

    geometry_msgs::msg::Pose
        cup_pose_;

    geometry_msgs::msg::Pose
        pot_world_pose_;

    geometry_msgs::msg::Pose
        pot_relative_pose_;

    geometry_msgs::msg::Quaternion
        carry_orientation_;

    tf2::Vector3 grasp_;
    tf2::Vector3 above_;
    tf2::Vector3 lift_;
    tf2::Vector3 cup_approach_;
    tf2::Vector3 pour_position_;

    std::vector<double>
        upright_joints_;

    double planning_time_=8.0;
    int planning_attempts_=15;

    double velocity_scaling_=0.15;
    double acceleration_scaling_=0.10;

    double position_tolerance_=0.008;
    double joint_tolerance_=0.03;

    double carry_orientation_tolerance_=0.55;

    double table_x_=0.10;
    double table_y_=0.0;
    double table_z_=-0.011;

    double table_sx_=0.38;
    double table_sy_=0.34;
    double table_sz_=0.02;

    double support_eps_=0.001;

    double cup_scale_=0.00085;
    double pot_scale_=0.001;
    double cal_cup_scale_=0.00085;

    double pour_delta_deg_=-15.0;

    bool pot_world_present_=false;
    bool pot_attached_=false;

    size_t step_=0;

    std::atomic<bool> busy_{false};
    std::atomic<bool> aborted_{false};

    bool failed_=false;
};

int main(
    int argc,
    char** argv)
{
    rclcpp::init(
        argc,
        argv
    );

    auto options=
        rclcpp::NodeOptions().
            automatically_declare_parameters_from_overrides(
                true
            );

    auto node=
        std::make_shared<
            rclcpp::Node
        >(
            "tea_moveit_app",
            options
        );

    rclcpp::executors::
        MultiThreadedExecutor
            executor(
                rclcpp::
                    ExecutorOptions(),
                4
            );

    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        }
    );

    auto task=
        std::make_shared<
            TeaTask
        >(node);

    if(!task->init()){
        rclcpp::shutdown();

        executor.cancel();

        if(spin_thread.joinable())
            spin_thread.join();

        return 1;
    }

    if(spin_thread.joinable())
        spin_thread.join();

    return 0;
}
