#ifndef LANDMARKS_STATUS_MANAGER_ALTERNATIVE_H
#define LANDMARKS_STATUS_MANAGER_ALTERNATIVE_H

#include "status_manager_multi_path.h"

namespace landmarks {
class StatusManagerFixedLama : public StatusManagerMultiPath {
    void collect_needed_again_relatives(const LandmarkNode *node,
                                        const State &state) override;
    void mark_reasonable_children_needed_again(
        const LandmarkNode *node);
public:
    explicit StatusManagerFixedLama(LandmarkGraph &graph, bool _add_goal_atoms, bool _add_gn_parents);
    ~StatusManagerFixedLama() override = default;
};
}

#endif
