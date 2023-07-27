#ifndef LANDMARKS_STATUS_MANAGER_LMCOUNT_H
#define LANDMARKS_STATUS_MANAGER_LMCOUNT_H

#include "landmark_status_manager.h"

namespace landmarks {
class StatusManagerLama : public LandmarkStatusManager {
    bool landmark_is_leaf(const LandmarkNode &node,
                          const BitsetView &accepted) const;
    bool landmark_needed_again(int id, const State &state);

public:
    explicit StatusManagerLama(LandmarkGraph &graph);
    ~StatusManagerLama() override = default;

    void update_lm_status(const State &ancestor_state) override;

    void progress_initial_state(const State &initial_state) override;
    void progress(const State &parent_ancestor_state,
                  OperatorID op_id,
                  const State &ancestor_state) override;
};
}

#endif
