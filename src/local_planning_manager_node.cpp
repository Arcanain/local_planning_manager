// src/local_planning_manager_node.cpp
#include "local_planning_manager/local_planning_manager_node.hpp"
#include <stdexcept>

// 実ノード名は当面固定（YAMLに合わせる）
static constexpr const char *PUREPURSUITNODE = "pure_pursuit_node";
static constexpr const char *DWANODE = "dwa_node";
static constexpr const char *STOPPINGNODE = "stopping_node";
static constexpr const char *INPLACETURNNODE = "inplace_turn_node";

namespace local_planning_manager
{
    // コンストラクタ
    LocalPlanningManagerNode::LocalPlanningManagerNode()
        : rclcpp::Node("local_planning_manager_node"),
        current_area_id_(""),
        current_state_id_(""),
        last_state_id_(""),           // ← これを先に
        area_index_(0),
        switch_threshold_(5.0),
        current_x_(0.0),
        current_y_(0.0),
        obstacle_detected_(false),
        closest_obstacle_distance_(0.0),  // ← これを後に
        state_entered_at_{}
    {
        // --- Parameters ---
        // const std::string graph_yaml_default = this->declare_parameter<std::string>("graph_yaml_path", "");
        // const std::string decider_yaml_default = this->declare_parameter<std::string>("decider_yaml_path", "");
        const double tick_hz = this->declare_parameter<double>("tick_hz", 2.0); // [Hz]

        // --- Subscribers ---
        using std::placeholders::_1;
        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&LocalPlanningManagerNode::gnssPoseCallback, this, _1));
        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", 10, std::bind(&LocalPlanningManagerNode::obstacleDetectedCallback, this, _1));
        closest_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/closest_point", 10, std::bind(&LocalPlanningManagerNode::closestPointCallback, this, _1));

        // --- Lifecycle Clients ---
        // change_state service clients
        dwa_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + DWANODE + "/change_state");
        purepursuit_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + PUREPURSUITNODE + "/change_state");
        stopping_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + STOPPINGNODE + "/change_state");
        inplace_turn_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + INPLACETURNNODE + "/change_state");

        // get_state service clients
        dwa_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + DWANODE + "/get_state");
        purepursuit_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + PUREPURSUITNODE + "/get_state");
        stopping_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + STOPPINGNODE + "/get_state");
        inplace_turn_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + INPLACETURNNODE + "/get_state");

        // --- Timer ---
        auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tick_hz)); // 周期[s]
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LocalPlanningManagerNode::timerCallback, this));
        last_tick_time_ = std::chrono::steady_clock::now();

        // --- Component ---
        LocalPlanningManagerComponent component_;

        // --- Initializations ---
        // this is temp //TODO: read from YAML or parameter
        area_list_.push_back("A");
        area_list_.push_back("B");
        area_list_.push_back("C");
        switch_threshold_ = 1.0; // [m]
        area_index_ = 0;
        current_area_id_ = area_list_[area_index_];
        area_switch_point_.emplace_back(10.0, 0.0); // Aへの切り替えポイント
        area_switch_point_.emplace_back(20.0, 0.0); // Bへの切り替えポイント
        area_switch_point_.emplace_back(30.0, 0.0); // Cへの切り替えポイント
    }

    // === Callbacks ===
    void LocalPlanningManagerNode::gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnssmsg)
    {
        current_x_ = gnssmsg->pose.position.x;
        current_y_ = gnssmsg->pose.position.y;
    }

    void LocalPlanningManagerNode::obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr obstaclemsg)
    {
        obstacle_detected_ = obstaclemsg->data;
    }

    void LocalPlanningManagerNode::closestPointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        const double x = msg->x;
        const double y = msg->y;
        closest_obstacle_distance_ = std::sqrt(x * x + y * y);
        RCLCPP_INFO(this->get_logger(), "Closest point: x=%.2f, y=%.2f, distance=%.2f",
                    x, y, closest_obstacle_distance_);
    }

    void LocalPlanningManagerNode::timerCallback()
    {
        /*処理内容
        常に行うこと
            1. SemanticStateを取得
            2. current_area_id_の変化条件判定

        erea_idごとの処理
        current_area_id_ == "A" (これは例)
            許可されている遷移：
                Pure Puresuit → 停止
                停止　→ Pure Puresuit
            行う条件判定：
                current_state == Pure_Pursuit
                    call 停止判定のメソッド
                current_state == 停止
                    call pure pursuit判定メソッド

        current_area_id_ == "B" (これは例)
            許可されている遷移：
                DWA → 停止
                停止 → DWA
                停止 → その場で旋回
        */

        // area_idの変化条件判定
        if (switchArea_(current_x_, current_y_))
        {
            area_index_ = (area_index_ + 1) % area_list_.size();
            current_area_id_ = area_list_[area_index_];
            RCLCPP_INFO(this->get_logger(), "Switched to area: %s", current_area_id_.c_str());
        }

        // SemanticStateを取得する
        current_semantic_ = getSemanticState_();
        current_area_id_ = getCurrentAreaId_();
        current_state_id_ = getCurrentStateId_();
        // closest_obstacle_distance_が更新されていない場合の誤作動を抑える→ infにしておく
        // closest_pointは，obstacle_detected_がtrueのときのみ更新される実装になっている．
        closest_obstacle_distance_ = std::numeric_limits<double>::infinity();
        /*
        if (!obstacle_detected_) {
            closest_obstacle_distance_ = std::numeric_limits<double>::infinity();
        }
        */

        const auto now = std::chrono::steady_clock::now();
        if (current_state_id_ != last_state_id_)
        {
            last_state_id_ = current_state_id_;
            //state_entered_at_ = now; // 単一の time_point を更新
            state_entered_at_[current_state_id_] = now; // 状態IDごとに記録
            RCLCPP_INFO(this->get_logger(), "[state entered] %s at t=%lld", last_state_id_.c_str(),
                        static_cast<long long>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()));
        }

        if (current_area_id_ == "A")
        {
            // 現在状態に入ってからの経過時間[s]
            const double elapsed_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - state_entered_at_[current_state_id_]).count();
            /*A:
                PurePuresuit -> Stopping (obstacle && d <= 0.5)
                PurePuresuit -> DWA      (obstacle && d <= 1.0)
                DWA          -> Stopping (obstacle && d <= 0.5)
                DWA          -> PurePuresuit (!obstacle)
                Stopping     -> PurePuresuit (!obstacle)
                Stopping     -> InplaceTurn (elapsed_s >= 10s)
                InplaceTurn  -> DWA      (InplaceTurnから5秒経過)
            */

            if (current_state_id_ == "PurePuresuit")
            {
                if (obstacle_detected_ && (closest_obstacle_distance_ <= 1.0))
                {
                    executeTransitionRecipe_(current_state_id_, "DWA");
                    return;
                }
                if (obstacle_detected_ && (closest_obstacle_distance_ <= 0.5))
                {
                    executeTransitionRecipe_(current_state_id_, "Stopping");
                    return;
                }
            }
            else if (current_state_id_ == "DWA")
            {
                if (obstacle_detected_ && (closest_obstacle_distance_ <= 0.5))
                {
                    executeTransitionRecipe_(current_state_id_, "Stopping");
                    return;
                }
                if (!obstacle_detected_)
                {
                    executeTransitionRecipe_(current_state_id_, "PurePuresuit");
                    return;
                }
            }
            else if (current_state_id_ == "Stopping")
            {
                if (!obstacle_detected_)
                {
                    executeTransitionRecipe_(current_state_id_, "PurePuresuit");
                    return;
                }
                if (elapsed_s >= 5.0)
                {
                    executeTransitionRecipe_(current_state_id_, "InplaceTurn");
                    return;
                }
            }
            else if (current_state_id_ == "InplaceTurn")
            {
                if (elapsed_s >= 1.0)
                {
                    executeTransitionRecipe_(current_state_id_, "DWA");
                    return;
                }
            }
        }
        else if (current_area_id_ == "B")
        {
            const double elapsed_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - state_entered_at_[current_state_id_]).count();
            /*
            purepursuit -> stopping
            stopping    -> purepursuit: 10s経過 or 障害物消失
            */
            if (current_state_id_ == "PurePuresuit")
            {
                if (obstacle_detected_ && (closest_obstacle_distance_ <= 0.5))
                {
                    executeTransitionRecipe_(current_state_id_, "Stopping");
                    return;
                }
            }
            else if (current_state_id_ == "Stopping")
            {
                if (!obstacle_detected_)
                {
                    executeTransitionRecipe_(current_state_id_, "PurePuresuit");
                    return;
                }
                if (elapsed_s >= 10.0)
                {
                    executeTransitionRecipe_(current_state_id_, "PurePuresuit");
                    return;
                }
            }
        }
        else if (current_area_id_ == "C")
        {
            /*
            purepursuit -> stopping
            stopping    -> purepursuit: 障害物消失
            */
            if (current_state_id_ == "PurePuresuit")
            {
                if (obstacle_detected_ && (closest_obstacle_distance_ <= 0.5))
                    executeTransitionRecipe_(current_state_id_, "Stopping");
            }
            else if (current_state_id_ == "Stopping")
            {
                if (!obstacle_detected_)
                    executeTransitionRecipe_(current_state_id_, "PurePuresuit");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown area: %s", current_area_id_.c_str());
        }
    }

    std::string LocalPlanningManagerNode::getCurrentAreaId_()
    {
        return current_area_id_;
    }

    std::string LocalPlanningManagerNode::getCurrentStateId_()
    {
        auto result = component_.getStateIdFromSemantic(current_semantic_);
        if (result.has_value())
        {
            return result.value();
        }
        return "Unknown";
    }

    bool LocalPlanningManagerNode::switchArea_(double from_x, double from_y)
    {
        auto [to_x, to_y] = area_switch_point_[area_index_];
        if (component_.isLessThanThreshold(from_x, from_y, to_x, to_y, 1.0))
            return true;
        return false;
    }

    SemanticState LocalPlanningManagerNode::getSemanticState_()
    {
        // 管理しているlifecycleノードの状態を取得し，SemanticStateを構築する
        SemanticState semantic;
        // 各ノードの状態を取得
        /*
        semantic.pure_pursuit_state = getNodeState_(purepursuit_get_client_);
        semantic.dwa_state = getNodeState_(dwa_get_client_);
        semantic.stopping_state = getNodeState_(stopping_get_client_);
        semantic.inplace_turn_state = getNodeState_(inplace_turn_get_client_);
        return semantic;
        */
        semantic.setNodeState("pure_pursuit_node",
                                static_cast<SemanticState::State>(getNodeState_(purepursuit_get_client_)));
        semantic.setNodeState("dwa_node",
                                static_cast<SemanticState::State>(getNodeState_(dwa_get_client_)));
        semantic.setNodeState("stopping_node",
                                static_cast<SemanticState::State>(getNodeState_(stopping_get_client_)));
        semantic.setNodeState("inplace_turn_node",
                                static_cast<SemanticState::State>(getNodeState_(inplace_turn_get_client_)));
        return semantic;
    }

    // ヘルパー関数：個別ノードの状態を取得
    uint8_t LocalPlanningManagerNode::getNodeState_(
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client)
    {
        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(request);

        // 同期的に待つ（タイムアウト付き）
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get()->current_state.id;
        }
        RCLCPP_WARN(this->get_logger(), "[LOG] LocalManager: Timeout. Failed to get node state");
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // --- 低レベル送信 ---
    bool LocalPlanningManagerNode::sendChange_(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        uint8_t transition_id,
        std::chrono::seconds timeout)
    {
        if (!client)
            return false;
        if (!client->wait_for_service(timeout))
        {
            RCLCPP_WARN(get_logger(), "ChangeState service not available");
            return false;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition_id;

        auto fut = client->async_send_request(req);
        auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, timeout);
        if (rc != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "ChangeState request timed out");
            return false;
        }
        auto resp = fut.get();
        if (!resp || !resp->success)
        {
            RCLCPP_WARN(get_logger(), "ChangeState request failed (transition id=%u)", transition_id);
            return false;
        }
        return true;
    }

    // --- 単純な activate/deactivate ヘルパ ---
    bool LocalPlanningManagerNode::activateNode_(const char *node_key)
    {
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client = nullptr;

        if (std::string(node_key) == PUREPURSUITNODE)
            client = purepursuit_lc_client_;
        else if (std::string(node_key) == DWANODE)
            client = dwa_lc_client_;
        else if (std::string(node_key) == STOPPINGNODE)
            client = stopping_lc_client_;
        else if (std::string(node_key) == INPLACETURNNODE)
            client = inplace_turn_lc_client_;
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown node key: %s", node_key);
            return false;
        }

        // 最小限：CONFIGURE → ACTIVATE（UNCONFIGURED/INACTIVE のどちらでも到達可能）
        const auto timeout = std::chrono::seconds(5);
        if (!sendChange_(client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout))
        {
            // 既に INACTIVE/ACTIVE の場合は CONFIGURE が失敗することがあるので、ACTIVATE だけ試みる
            RCLCPP_DEBUG(get_logger(), "[%s] CONFIGURE failed; try ACTIVATE only", node_key);
        }
        if (!sendChange_(client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout))
        {
            RCLCPP_ERROR(get_logger(), "[%s] ACTIVATE failed", node_key);
            return false;
        }
        return true;
    }

    bool LocalPlanningManagerNode::deactivateNode_(const char *node_key)
    {
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client = nullptr;

        if (std::string(node_key) == PUREPURSUITNODE)
            client = purepursuit_lc_client_;
        else if (std::string(node_key) == DWANODE)
            client = dwa_lc_client_;
        else if (std::string(node_key) == STOPPINGNODE)
            client = stopping_lc_client_;
        else if (std::string(node_key) == INPLACETURNNODE)
            client = inplace_turn_lc_client_;
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown node key: %s", node_key);
            return false;
        }

        // 最小限：DEACTIVATE（ACTIVE でなければ失敗することもあるが、ここでは単純化）
        const auto timeout = std::chrono::seconds(5);
        if (!sendChange_(client, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, timeout))
        {
            RCLCPP_DEBUG(get_logger(), "[%s] DEACTIVATE failed (maybe already inactive)", node_key);
        }
        return true;
    }

    // --- 9通りの遷移本体（必要なノードだけ触る。冗長でもOK方針） ---

    bool LocalPlanningManagerNode::transitionPurePuresuitToDwa_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && activateNode_(DWANODE);
        // 念のため他は落としておく（冗長でOK）
        ok = ok && deactivateNode_(STOPPINGNODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionPurePuresuitToStop_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && activateNode_(STOPPINGNODE);
        ok = ok && deactivateNode_(DWANODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionDwaToStop_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(DWANODE);
        ok = ok && activateNode_(STOPPINGNODE);
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionDwaToPurePuresuit_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(DWANODE);
        ok = ok && activateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(STOPPINGNODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionStopToPurePuresuit_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(STOPPINGNODE);
        ok = ok && activateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(DWANODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionStopToDwa_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(STOPPINGNODE);
        ok = ok && activateNode_(DWANODE);
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(INPLACETURNNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionStopToInplaceTurn_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(STOPPINGNODE);
        ok = ok && activateNode_(INPLACETURNNODE);
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(DWANODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionInplaceTurnToPurePuresuit_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(INPLACETURNNODE);
        ok = ok && activateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(DWANODE);
        ok = ok && deactivateNode_(STOPPINGNODE);
        return ok;
    }

    bool LocalPlanningManagerNode::transitionInplaceTurnToDwa_()
    {
        bool ok = true;
        ok = ok && deactivateNode_(INPLACETURNNODE);
        ok = ok && activateNode_(DWANODE);
        ok = ok && deactivateNode_(PUREPURSUITNODE);
        ok = ok && deactivateNode_(STOPPINGNODE);
        return ok;
    }

    // --- from/to（StateId）で分岐して実行 ---
    void LocalPlanningManagerNode::executeTransitionRecipe_(const std::string &from, const std::string &to)
    {
        if (from == to)
        {
            RCLCPP_INFO(get_logger(), "executeTransitionRecipe_: from==to (%s). No-op.", from.c_str());
            return;
        }

        bool ok = false;

        // from PUREPURSUIT
        if (from == "PurePuresuit" && to == "DWA")
            ok = transitionPurePuresuitToDwa_();
        else if (from == "PurePuresuit" && to == "Stopping")
            ok = transitionPurePuresuitToStop_();

        // from DWA
        else if (from == "DWA" && to == "Stopping")
            ok = transitionDwaToStop_();
        else if (from == "DWA" && to == "PurePuresuit")
            ok = transitionDwaToPurePuresuit_();

        // from STOPPING
        else if (from == "Stopping" && to == "PurePuresuit")
            ok = transitionStopToPurePuresuit_();
        else if (from == "Stopping" && to == "DWA")
            ok = transitionStopToDwa_();
        else if (from == "Stopping" && to == "InplaceTurn")
            ok = transitionStopToInplaceTurn_();

        // from INPLACETURN
        else if (from == "InplaceTurn" && to == "PurePuresuit")
            ok = transitionInplaceTurnToPurePuresuit_();
        else if (from == "InplaceTurn" && to == "DWA")
            ok = transitionInplaceTurnToDwa_();

        // Undefined transition
        else
        {
            RCLCPP_ERROR(get_logger(), "executeTransitionRecipe_: undefined transition %s -> %s",
                         from.c_str(), to.c_str());
            return;
        }

        if (ok)
        {
            RCLCPP_INFO(get_logger(), "State transition completed: %s -> %s", from.c_str(), to.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "State transition failed: %s -> %s", from.c_str(), to.c_str());
        }
    }

}
