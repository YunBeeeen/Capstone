#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit/robot_state/conversions.h>

class CollisionScene : public rclcpp::Node {
public:
  CollisionScene() : Node("moveit_collision_scene") {
    const auto urdf_text=declare_parameter<std::string>("robot_description", "");
    const auto srdf_text=declare_parameter<std::string>("robot_description_semantic", "");
    auto urdf=urdf::parseURDF(urdf_text);
    auto srdf=std::make_shared<srdf::Model>();
    if(!urdf || !srdf->initString(*urdf,srdf_text)) throw std::runtime_error("Invalid URDF/SRDF");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    scene_=std::make_shared<planning_scene::PlanningScene>(model);
    scene_->getCurrentStateNonConst().setToDefaultValues();
    scene_->getCurrentStateNonConst().update();
    semantic_=create_publisher<std_msgs::msg::String>("robot_description_semantic",rclcpp::QoS(1).transient_local());
    std_msgs::msg::String semantic_msg;semantic_msg.data=srdf_text;semantic_->publish(semantic_msg);
    scene_pub_=create_publisher<moveit_msgs::msg::PlanningScene>("monitored_planning_scene",rclcpp::QoS(1).transient_local());
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>("joint_states",10,[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      auto& state=scene_->getCurrentStateNonConst();
      const auto& names=state.getRobotModel()->getVariableNames();
      for(size_t i=0;i<msg->name.size() && i<msg->position.size();++i)
        if(std::isfinite(msg->position[i]) && std::find(names.begin(),names.end(),msg->name[i])!=names.end()) state.setVariablePosition(msg->name[i],msg->position[i]);
      state.update();
    });
    get_=create_service<moveit_msgs::srv::GetPlanningScene>("get_planning_scene",[this](const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> req,std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> res){scene_->getPlanningSceneMsg(res->scene,req->components);});
    apply_=create_service<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene",[this](const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res){res->success=scene_->usePlanningSceneMsg(req->scene);});
    validity_=create_service<moveit_msgs::srv::GetStateValidity>("check_state_validity",[this](const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Request> req,std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> res){
      moveit::core::RobotState state(scene_->getCurrentState());
      moveit::core::robotStateMsgToRobotState(req->robot_state,state,true);state.update();
      res->valid=scene_->isStateValid(state,req->constraints,req->group_name);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(500),[this](){
      moveit_msgs::msg::PlanningScene msg;scene_->getPlanningSceneMsg(msg);scene_pub_->publish(msg);
      collision_detection::CollisionRequest req;collision_detection::CollisionResult result;
      scene_->checkCollision(req,result);
      if(result.collision) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Current state is in collision (adjacent arm pairs excluded).");
    });
    RCLCPP_INFO(get_logger(),"Loaded %s: %zu adjacent pairs excluded; /check_state_validity and /get_planning_scene ready.",model->getName().c_str(),srdf->getDisabledCollisionPairs().size());
  }
private:
  planning_scene::PlanningScenePtr scene_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  rclcpp::Service<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try {rclcpp::spin(std::make_shared<CollisionScene>());}catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("moveit_collision_scene"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
