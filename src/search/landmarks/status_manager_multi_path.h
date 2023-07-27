#ifndef LANDMARKS_STATUS_MANAGER_MULTI_PATH_H
#define LANDMARKS_STATUS_MANAGER_MULTI_PATH_H

#include "landmark_status_manager.h"

namespace landmarks {
class StatusManagerMultiPath : public LandmarkStatusManager {
protected:
    std::vector<short> reached_lms;
    bool add_goal_atoms;
    bool add_gn_parents;

    void reset_reached_lms();
    bool landmark_is_true_in_state(int id, const State &state);
    virtual void collect_needed_again_relatives(
        const LandmarkNode *node, const State &state);
    void mark_greedy_necessary_parents_needed_again(
        const LandmarkNode *node, const State &state);
public:
    explicit StatusManagerMultiPath(LandmarkGraph &graph, bool _add_goal_atoms, bool _add_gn_parents);
    ~StatusManagerMultiPath() override = default;

    void update_lm_status(const State &ancestor_state) override;

    void progress_initial_state(const State &initial_state) override;
    void progress(const State &parent_ancestor_state,
                  OperatorID op_id,
                  const State &ancestor_state) override;
};
}

#endif
