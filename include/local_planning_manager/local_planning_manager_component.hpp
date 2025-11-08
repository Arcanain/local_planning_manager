#pragma once
#include <string>
#include <optional>
#include <unordered_map>
#include <cstdint>

namespace local_planning_manager
{
    // ============================================
    // 状態の意味的側面を表す構造体
    // ============================================
    struct SemanticState
    {
        /*
        string: node_name
        State:  tuple of node states
        */
        enum class State : std::uint8_t
        {
            UNKNOWN = 0,
            UNCONFIGURED = 1,
            INACTIVE = 2,
            ACTIVE = 3,
            FINALIZED = 4
        };

        std::unordered_map<std::string, State> semanticState;

        bool operator==(const SemanticState &other) const noexcept;

        // 便利メソッド
        bool isNodeActive(const std::string &nodeName) const noexcept;
        void setNodeState(const std::string &nodeName, State state);
    };

    // ============================================
    // 状態管理コンポーネント本体
    // ============================================
    class LocalPlanningManagerComponent
    {
    public:
        LocalPlanningManagerComponent();

        // SemanticStateから対応するstateIdを取得
        std::optional<std::string> getStateIdFromSemantic(const SemanticState& semantic) const;


        // 2点間の距離が閾値以下ならtrue
        bool isLessThanThreshold(
            double from_x, double from_y,
            double to_x, double to_y,
            double threshold) const;

        // 2点間の距離を計算
        double calculateDistance(
            double from_x, double from_y,
            double to_x, double to_y) const;

    private:
        // stateId → SemanticState のマッピング
        std::unordered_map<std::string, SemanticState> state_map_;

        // マッピングの初期化
        void initializeStateMapping();
        

    };
    

} // namespace local_planning_manager