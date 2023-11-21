#ifndef LANDMARKS_STATUS_MANAGER_MULTI_PATH_CONSISTENT_H
#define LANDMARKS_STATUS_MANAGER_MULTI_PATH_CONSISTENT_H

#include "landmark_status_manager.h"

namespace landmarks {
class StatusManagerMultiPathConsistent : public LandmarkStatusManager {
protected:
    //std::vector<short> reached_lms;
    bool add_goal_atoms;
    bool add_gn_parents;
    bool add_reasonalbe_children;
    PerStateBitset required_lms;

    void mark_required_again_relatives(
        const LandmarkNode *node, const State &state,
        BitsetView &required_landmarks);
    void set_greedy_necessary_parents_required(
        const LandmarkNode *node, const State &state, BitsetView &required);
    void set_reasonable_children_required(
        const LandmarkNode *node, BitsetView &required);
public:
    explicit StatusManagerMultiPathConsistent(
        LandmarkGraph &graph, bool add_goal_atoms,
        bool add_gn_parents, bool add_reasonable_children);
    ~StatusManagerMultiPathConsistent() override = default;

    void update_lm_status(const State &ancestor_state) override;

    void set_landmarks_for_initial_state(const State &initial_state) override;
    bool update_accepted_landmarks(
        const State &parent_ancestor_state, OperatorID op_id,
        const State &ancestor_state) override;
};
}

#endif
