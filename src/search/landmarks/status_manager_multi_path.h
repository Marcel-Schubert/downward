#ifndef LANDMARKS_STATUS_MANAGER_LM_ASTAR_H
#define LANDMARKS_STATUS_MANAGER_LM_ASTAR_H

#include "landmark_status_manager.h"

namespace landmarks {
class StatusManagerMultiPath : public LandmarkStatusManager {
protected:
    std::vector<short> reached_lms;
    bool add_goal_atoms;
    bool add_gn_parents;
    bool add_reasonalbe_children;

    void reset_reached_lms();
    virtual void collect_needed_again_relatives(
        const LandmarkNode *node, const State &state);
    void mark_greedy_necessary_parents_needed_again(
        const LandmarkNode *node, const State &state);
    void mark_reasonable_children_needed_again(const LandmarkNode *node);
public:
    explicit StatusManagerMultiPath(
        LandmarkGraph &graph, bool add_goal_atoms,
        bool add_gn_parents, bool add_reasonable_children);
    ~StatusManagerMultiPath() override = default;

    void update_lm_status(const State &ancestor_state) override;

    void set_landmarks_for_initial_state(const State &initial_state) override;
    bool update_accepted_landmarks(
        const State &parent_ancestor_state, OperatorID op_id,
        const State &ancestor_state) override;
};
}

#endif
