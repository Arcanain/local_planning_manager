#include "local_planning_manager/local_planning_manager_component.hpp"
#include <stdexcept>
#include <cmath> 

static constexpr const char *PURE_PURSUIT_NODE   = "pure_pursuit_node";
static constexpr const char *DWA_NODE            = "dwa_node";
static constexpr const char *STOP_MOTION_NODE    = "stop_motion_node";     // 修正
static constexpr const char *IN_PLACE_TURN_NODE  = "in_place_turn_node";   ;

namespace local_planning_manager
{
    // ============================================
    // SemanticState
    // ============================================
    bool SemanticState::operator==(const SemanticState &other) const noexcept
    {
        return semanticState == other.semanticState;
    }

    bool SemanticState::isNodeActive(const std::string &nodeName) const noexcept
    {
        auto it = semanticState.find(nodeName);
        return it != semanticState.end() && it->second == State::ACTIVE;
    }

    void SemanticState::setNodeState(const std::string &nodeName, State state)
    {
        semanticState[nodeName] = state;
    }

    // ============================================
    // LocalPlanningManagerComponent
    // ============================================
    
    // コンストラクタ
    LocalPlanningManagerComponent::LocalPlanningManagerComponent()
    {
        initializeStateMapping();
    }

    // マッピングの初期化
    // TODO  yamlからの読み取りに変更する
    void LocalPlanningManagerComponent::initializeStateMapping()
    {
        // 各stateIdに対応するSemanticStateを登録

        SemanticState purepuresuitState;
        purepuresuitState.setNodeState(PURE_PURSUIT_NODE,  SemanticState::State::ACTIVE);
        purepuresuitState.setNodeState(DWA_NODE,          SemanticState::State::INACTIVE);
        purepuresuitState.setNodeState(STOP_MOTION_NODE,     SemanticState::State::INACTIVE);
        purepuresuitState.setNodeState(IN_PLACE_TURN_NODE,  SemanticState::State::INACTIVE);
        state_map_["PurePuresuit"] = purepuresuitState;      
        
        SemanticState dwaState;
        dwaState.setNodeState(DWA_NODE,         SemanticState::State::ACTIVE);
        dwaState.setNodeState(PURE_PURSUIT_NODE, SemanticState::State::INACTIVE);
        dwaState.setNodeState(STOP_MOTION_NODE,    SemanticState::State::INACTIVE);
        dwaState.setNodeState(IN_PLACE_TURN_NODE, SemanticState::State::INACTIVE); 
        state_map_["DWA"] = dwaState;

        SemanticState stoppingState;
        stoppingState.setNodeState(PURE_PURSUIT_NODE, SemanticState::State::INACTIVE);
        stoppingState.setNodeState(DWA_NODE,         SemanticState::State::INACTIVE);
        stoppingState.setNodeState(STOP_MOTION_NODE,    SemanticState::State::ACTIVE);
        stoppingState.setNodeState(IN_PLACE_TURN_NODE, SemanticState::State::INACTIVE);
        state_map_["Stopping"] = stoppingState;

        SemanticState inplaceTurnState;
        inplaceTurnState.setNodeState(PURE_PURSUIT_NODE, SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(DWA_NODE,         SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(STOP_MOTION_NODE,    SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(IN_PLACE_TURN_NODE, SemanticState::State::ACTIVE); 
        state_map_["InplaceTurn"] = inplaceTurnState;

        // 必要に応じて他の状態も追加
    }

    // SemanticStateから対応するstateIdを取得
    std::optional<std::string> LocalPlanningManagerComponent::getStateIdFromSemantic(
        const SemanticState &semantic) const
    {
        // マッピングを逆引き
        for (const auto &[id, registeredSemantic] : state_map_)
        {
            if (registeredSemantic == semantic)
            {
                return id;
            }
        }
        return std::nullopt; // 見つからなかった
    }



    // 2点間の距離を計算
    double LocalPlanningManagerComponent::calculateDistance(
        double from_x, double from_y,
        double to_x, double to_y) const
    {
        double dx = to_x - from_x;
        double dy = to_y - from_y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 2点間の距離が閾値以下ならtrue
    bool LocalPlanningManagerComponent::isLessThanThreshold(
        double from_x, double from_y,
        double to_x, double to_y,
        double threshold) const
    {
        double distance = calculateDistance(from_x, from_y, to_x, to_y);
        return distance <= threshold;
    }

} // namespace local_planning_manager