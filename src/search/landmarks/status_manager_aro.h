#ifndef LANDMARKS_STATUS_MANAGER_ARO_H
#define LANDMARKS_STATUS_MANAGER_ARO_H

#include "landmark_status_manager.h"

namespace landmarks {
class StatusManagerARO : public LandmarkStatusManager {
    PerStateBitset future_lms;

    void progress_basic(
        const BitsetView &parent_past, const BitsetView &parent_fut,
        const State &parent_ancestor_state, BitsetView &past, BitsetView &fut,
        const State &ancestor_state);
    void progress_goal(int id, const State &ancestor_state, BitsetView &fut);
    void progress_greedy_necessary(int id, const State &ancestor_state,
                                   const BitsetView &past, BitsetView &fut);
    void progress_reasonable(int id, const BitsetView &past, BitsetView &fut);
public:
    explicit StatusManagerARO(LandmarkGraph &lm_graph);

    virtual void update_lm_status(const State &ancestor_state) override;

    virtual void progress_initial_state(const State &initial_state) override;
    virtual void progress(
        const State &parent_ancestor_state, OperatorID op_id,
        const State &ancestor_state) override;
};
}

#endif
