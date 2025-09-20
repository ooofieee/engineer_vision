#include "rclcpp/rclcpp.hpp"
#include "moveit/task_constructor/container.h"
#include "moveit/task_constructor/task.h"
#include "moveit/task_constructor/solvers.h"
#include "moveit/task_constructor/stages.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace mtc = moveit::task_constructor;

class MTC
{
public:
    MTC(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
        RCLCPP_INFO(node->get_logger(), "MTC launched");
        task_ = createTask();
    }

    void doTask()
    {
        task_ = createTask();
        try
        {
            task_.init();
        }
        catch (moveit::task_constructor::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(logger, e);
            return;
        }

        if (!task_.plan(5))
        {
            RCLCPP_ERROR_STREAM(logger, "Task planning failed");
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(logger, "Task execution failed");
            return;
        }

        return;
    }

    moveit::task_constructor::Task createTask()
    {
        mtc::Task task;
        task.stages()->setName("demo task");
        task.loadRobotModel(node_);

        const auto &arm_group_name = "panda_arm";
        const auto &hand_group_name = "hand";
        const auto &hand_frame = "panda_hand";

        // Set task properties
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);
        // 该阶段计划移动到 "张开的手 "姿势，这是 SRDF 中为机器人定义的指定姿势。
        // clang-format off
        auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        // clang-format on
        stage_open_hand->setGroup(hand_group_name);
        stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));
        // 连接阶段
        // clang-format off
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
        // clang-format on
        stage_move_to_pick->setTimeout(5.0);
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));

        // clang-format off
        mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
        // clang-format on

        // This is an example of SerialContainer usage. It's not strictly needed here.
        // In fact, `task` itself is a SerialContainer by default.
        {
            // 抓取的串行容器，包含很多子阶段
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"}); // exposeTo() 在新的串行容器中声明父任务的任务属性
            // clang-format off
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });  // configureInitFrom() 对其进行初始化。 这样，所包含的阶段就可以访问这些属性。
            // clang-format on

            {
                // clang-format off
                auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);  // 传播阶段，允许我们指定从当前位置开始的相对移动。
                // clang-format on
                stage->properties().set("marker_ns", "approach_object");
                stage->properties().set("link", hand_frame);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.1, 0.15); // 设定最小和最大移动距离

                // Set hand forward direction(设置手部要达到的位姿)
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = hand_frame;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }

            /****************************************************
          ---- *               Generate Grasp Pose                *
             ***************************************************/
            {
                // Sample grasp pose
                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose"); // 生成抓取位姿阶段
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject("object");
                stage->setAngleDelta(M_PI / 12);             // 设置采样角度，delta 越小，抓取方向就越接近，不同方向的抓取越多
                stage->setMonitoredStage(current_state_ptr); // Hook into current state

                // This is the transform from the object frame to the end-effector frame
                // 生成末端执行器位姿，也可以使用 geometry_msgs 的 PoseStamped
                Eigen::Isometry3d grasp_frame_transform;
                Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
                grasp_frame_transform.linear() = q.matrix();
                grasp_frame_transform.translation().z() = 0.1;

                // Compute IK
                // clang-format off
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage)); // 逆运动学计算阶段
                // clang-format on
                wrapper->setMaxIKSolutions(8);        // 有些机器人对给定姿势有多个逆运动学求解方案, 这里求解方案数量限制为 8 个
                wrapper->setMinSolutionDistance(1.0); // 设置最小求解距离，这是求解差异的阈值：如果某个求解的关节位置与之前的求解过于相似，则该求解将被标记为无效
                wrapper->setIKFrame(grasp_frame_transform, hand_frame);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                grasp->insert(std::move(wrapper));
            }

            {
                // 关闭碰撞检测阶段
                // clang-format off
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
                // clang-format on
                grasp->insert(std::move(stage));
            }

            {
                // 夹阶段
                auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
                stage->setGroup(hand_group_name);
                stage->setGoal("close");
                grasp->insert(std::move(stage));
            }

            {
                // 再次使用ModifyPlanningScene, 并使用 attachObject 将对象附加到手部
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                stage->attachObject("object", hand_frame);
                attach_object_stage = stage.get();
                grasp->insert(std::move(stage));
            }

            {
                // 抬起阶段
                // clang-format off
                auto stage =std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
                // clang-format on
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.1, 0.3);
                stage->setIKFrame(hand_frame);
                stage->properties().set("marker_ns", "lift_object");

                // Set upward direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
            }
            task.add(std::move(grasp)); // 把pick阶段添加到任务中
        }

        {
            // 放置连接阶段
            // clang-format off
            auto stage_move_to_place = std::make_unique<mtc::stages::Connect>("move to place",
                mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                          { hand_group_name, interpolation_planner } });
            // clang-format on
            stage_move_to_place->setTimeout(5.0);
            stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
            task.add(std::move(stage_move_to_place));
        }

        {
            // 放置串行容器
            auto place = std::make_unique<mtc::SerialContainer>("place object");
            task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
            // clang-format off
            place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
            // clang-format on

            /****************************************************
          ---- *               Generate Place Pose                *
             ***************************************************/
            {
                // Sample place pose
                // 生成放置位姿
                auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "place_pose");
                stage->setObject("object");

                geometry_msgs::msg::PoseStamped target_pose_msg;
                target_pose_msg.header.frame_id = "object";
                target_pose_msg.pose.position.y = 0.5;
                target_pose_msg.pose.orientation.w = 1.0;
                stage->setPose(target_pose_msg);
                // 我们使用 setMonitoredStage，并将先前 attach_object 阶段的指针传递给它。 这样，该阶段就能知道对象是如何附加的了
                stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

                // Compute IK
                // clang-format off
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));  // IK计算阶段
                // clang-format on
                wrapper->setMaxIKSolutions(2);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame("object");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                place->insert(std::move(wrapper));
            }

            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner); // 打开手部阶段
                stage->setGroup(hand_group_name);
                stage->setGoal("open");
                place->insert(std::move(stage));
            }

            {
                // clang-format off
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");  // 打开禁止碰撞阶段
                stage->allowCollisions("object",
                task.getRobotModel()
                    ->getJointModelGroup(hand_group_name)
                    ->getLinkModelNamesWithCollisionGeometry(),
                false);
                // clang-format on
                place->insert(std::move(stage));
            }

            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object"); // 分离对象阶段
                stage->detachObject("object", hand_frame);
                place->insert(std::move(stage));
            }

            {
                // 我们使用 "相对移动 "阶段从物体上退下，这与接近物体和提升物体阶段的操作类似。
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.1, 0.3);
                stage->setIKFrame(hand_frame);
                stage->properties().set("marker_ns", "retreat");

                // Set retreat direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = "world";
                vec.vector.x = -0.5;
                stage->setDirection(vec);
                place->insert(std::move(stage));
            }
            task.add(std::move(place)); // 将放置阶段添加到任务中
        }

        {
            // 归零阶段，SRDF中定义的 ready 状态
            auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setGoal("ready");
            task.add(std::move(stage));
        }
        return task;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger = node_->get_logger();
    moveit::task_constructor::Task task_;
};