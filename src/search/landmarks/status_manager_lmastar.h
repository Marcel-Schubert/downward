#ifndef LANDMARKS_STATUS_MANAGER_LMASTAR_H
#define LANDMARKS_STATUS_MANAGER_LMASTAR_H

#include "landmark_status_manager.h"

#include "../task_proxy.h"

namespace landmarks {
class LandmarkGraph;

class StatusManagerLMAstar : public LandmarkStatusManager {
    void progress_basic(
        const BitsetView &parent_past, const State &parent_ancestor_state,
        BitsetView &past, const State &ancestor_state);

    void progress_goal(int id, const State &ancestor_state);
    void progress_greedy_necessary(int id, const State &ancestor_state);

public:
    explicit StatusManagerLMAstar(LandmarkGraph &lm_graph);

    virtual void update_lm_status(const State &ancestor_state) override;

    virtual void progress_initial_state(const State &initial_state) override;
    virtual void progress(
        const State &parent_ancestor_state, OperatorID op_id,
        const State &ancestor_state) override;
};
}

#endif
