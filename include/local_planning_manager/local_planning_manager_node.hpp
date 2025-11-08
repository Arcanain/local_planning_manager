// include/local_planning_manager/local_plannning_manager_node.hpp

#pragma once
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <std_msgs/msg/bool.hpp>
// ROS2 library
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Lifecycle library
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// Local Planning Manager library
#include "local_planning_manager/local_planning_manager_component.hpp"

namespace local_planning_manager
{

    class LocalPlanningManagerNode : public rclcpp::Node
    {
    public:
        LocalPlanningManagerNode();
        ~LocalPlanningManagerNode() = default;

    private:



        // === ROS Subscribers ===
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr closest_point_sub_;

        
        // === ROS Subscriber Callbacks ===
        void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnssmsg);
        void obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr obstaclemsg);
        void closestPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
        void timerCallback();

        // --- Lifecycle Clients ---        
        // change_state service clients
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr dwa_lc_client_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr purepursuit_lc_client_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr stopping_lc_client_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr inplace_turn_lc_client_;
        // get_state service clients
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    dwa_get_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    purepursuit_get_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    stopping_get_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    inplace_turn_get_client_;   
        
        
        // -- Timer ---
        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::steady_clock::time_point last_tick_time_;
        
        // --- Component ---
        LocalPlanningManagerComponent component_;  // TODO:コンストラクタなど，
        //TODO: そもそもこいつは初め何も持っていないがこれは大丈夫なのか？
    
        // === Privete Methods ===
        SemanticState getSemanticState_(); // 各Lifecycleノードのactive状態を取得

        std::string   getCurrentAreaId_();
        std::string   getCurrentStateId_();
        bool          switchArea_(double x, double y);
        uint8_t       getNodeState_(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client);
        
        void executeTransitionRecipe_(const std::string& from_id, const std::string& to_id);
        
        // 低レベル送信ヘルパ
        bool sendChange_(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client, uint8_t transition_id, std::chrono::seconds timeout);
        
        // 単純な activate/deactivate ヘルパ（現在状態は見ずに必要最小限を送る）
        bool activateNode_(const char* node_key);    // CONFIGURE → ACTIVATE
        bool deactivateNode_(const char* node_key);  // DEACTIVATE

        // 9通りの遷移（StateIdに対応）
        bool transitionPurePuresuitToDwa_();
        bool transitionPurePuresuitToStop_();

        bool transitionDwaToStop_();
        bool transitionDwaToPurePuresuit_();

        bool transitionStopToPurePuresuit_();
        bool transitionStopToDwa_();
        bool transitionStopToInplaceTurn_();

        bool transitionInplaceTurnToPurePuresuit_();
        bool transitionInplaceTurnToDwa_();

        // === Private Members ===
        SemanticState            current_semantic_;
        std::string              current_area_id_;
        std::string              current_state_id_;
        std::string              last_state_id_;
        std::vector<std::string> area_list_;
        int                      area_index_;
        std::vector<std::tuple<double, double>> area_switch_point_;
        double switch_threshold_;
        double current_x_;
        double current_y_;
        bool   obstacle_detected_;
        double closest_obstacle_distance_;
        
        std::unordered_map<std::string, std::chrono::steady_clock::time_point> state_entered_at_;

    };

} //// namespace local_planning_manager
