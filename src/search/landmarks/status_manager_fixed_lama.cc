#include "status_manager_fixed_lama.h"

using namespace std;

namespace landmarks {
StatusManagerFixedLama::StatusManagerFixedLama(LandmarkGraph &graph, bool _add_goal_atoms, bool _add_gn_parents)
    : StatusManagerMultiPath(graph, _add_goal_atoms, _add_gn_parents) {
}

void StatusManagerFixedLama::collect_needed_again_relatives(
    const LandmarkNode *node, const State &state) {
    StatusManagerMultiPath::collect_needed_again_relatives(node, state);
    mark_reasonable_children_needed_again(node);
}

void StatusManagerFixedLama::mark_reasonable_children_needed_again(
    const LandmarkNode *node) {
    /*
      For all A -r-> B where A is not first added but B is, B must be
      destroyed to achieve A (definition of reasonable orderings).
      Therefore B is needed again.
    */

    for (auto it = node->reasonable_children_begin(); it != node->reasonable_children_end(); ++it) {
        int child_id = (*it)->get_id();
        if (lm_status[child_id] == PAST) {
            lm_status[child_id] = PAST_AND_FUTURE;
        }
    }
}
}

