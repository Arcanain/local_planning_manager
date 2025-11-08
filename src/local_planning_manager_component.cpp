#include "local_planning_manager/local_planning_manager_component.hpp"
#include <stdexcept>
#include <cmath> 

static constexpr const char *PUREPURSUITNODE = "pure_pursuit_node";
static constexpr const char *DWANODE = "dwa_node";
static constexpr const char *STOPPINGNODE = "stopping_node";
static constexpr const char *INPLACETURNNODE = "inplace_turn_node";

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
        purepuresuitState.setNodeState(PUREPURSUITNODE,  SemanticState::State::ACTIVE);
        purepuresuitState.setNodeState(DWANODE,          SemanticState::State::INACTIVE);
        purepuresuitState.setNodeState(STOPPINGNODE,     SemanticState::State::INACTIVE);
        purepuresuitState.setNodeState(INPLACETURNNODE,  SemanticState::State::INACTIVE);
        state_map_["PurePuresuit"] = purepuresuitState;      
        
        SemanticState dwaState;
        dwaState.setNodeState(DWANODE,         SemanticState::State::ACTIVE);
        dwaState.setNodeState(PUREPURSUITNODE, SemanticState::State::INACTIVE);
        dwaState.setNodeState(STOPPINGNODE,    SemanticState::State::INACTIVE);
        dwaState.setNodeState(INPLACETURNNODE, SemanticState::State::INACTIVE); 
        state_map_["DWA"] = dwaState;

        SemanticState stoppingState;
        stoppingState.setNodeState(PUREPURSUITNODE, SemanticState::State::INACTIVE);
        stoppingState.setNodeState(DWANODE,         SemanticState::State::INACTIVE);
        stoppingState.setNodeState(STOPPINGNODE,    SemanticState::State::ACTIVE);
        stoppingState.setNodeState(INPLACETURNNODE, SemanticState::State::INACTIVE);
        state_map_["Stopping"] = stoppingState;

        SemanticState inplaceTurnState;
        inplaceTurnState.setNodeState(PUREPURSUITNODE, SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(DWANODE,         SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(STOPPINGNODE,    SemanticState::State::INACTIVE);
        inplaceTurnState.setNodeState(INPLACETURNNODE, SemanticState::State::ACTIVE); 
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