#include "status_manager_lama.h"

using namespace std;

namespace landmarks {
StatusManagerLama::StatusManagerLama(LandmarkGraph &graph)
    : LandmarkStatusManager(graph) {
}

void StatusManagerLama::progress_initial_state(
        const State &initial_state) {
    BitsetView accepted = past_lms[initial_state];

    for (auto &node : lm_graph.get_nodes()) {
        if (!node->is_true_in_state(initial_state)
            || !node->final_parents.empty()) {
            accepted.reset(node->get_id());
        }
    }
}

void StatusManagerLama::progress(
        const State &parent_ancestor_state, OperatorID,
        const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return;
    }

    const BitsetView parent_accepted = past_lms[parent_ancestor_state];
    BitsetView accepted = past_lms[ancestor_state];

    int num_landmarks = lm_graph.get_num_landmarks();
    assert(accepted.size() == num_landmarks);
    assert(parent_accepted.size() == num_landmarks);

    /*
       Set all landmarks not accepted by this parent as "not accepted".
       Over multiple paths, this has the effect of computing the intersection
       of "accepted" for the parents. It is important here that upon first visit,
       all elements in "accepted" are true because true is the neutral element
       of intersection.

       In the case where the landmark we are setting to false here is actually
       achieved right now, it is set to "true" again below.
    */
    accepted.intersect(parent_accepted);

    // Mark landmarks accepted right now as "accepted" (if they are "leaves").
    for (int id = 0; id < num_landmarks; ++id) {
        if (!accepted.test(id)) {
            LandmarkNode *node = lm_graph.get_landmark(id);
            if (node->is_true_in_state(ancestor_state)) {
                if (landmark_is_leaf(*node, accepted)) {
                    accepted.set(id);
                }
            }
        }
    }
}

void StatusManagerLama::update_lm_status(const State &ancestor_state) {
    const BitsetView accepted = past_lms[ancestor_state];
    const int num_landmarks = lm_graph.get_num_landmarks();

    /* This first loop is necessary as setup for the *needed again*
       check in the second loop. */
    for (int id = 0; id < num_landmarks; ++id) {
        lm_status[id] = accepted.test(id) ? PAST : FUTURE;
    }
    for (int id = 0; id < num_landmarks; ++id) {
        if (lm_status[id] == PAST
            && landmark_needed_again(id, ancestor_state)) {
            lm_status[id] = PAST_AND_FUTURE;
        }
    }
}

bool StatusManagerLama::landmark_needed_again(
    int id, const State &state) {
    LandmarkNode *node = lm_graph.get_landmark(id);
    if (node->is_true_in_state(state)) {
        return false;
    } else if (node->is_true_in_goal) {
        return true;
    } else {
        /*
          For all A ->_gn B, if B is not accepted and A currently not
          true, since A is a necessary precondition for actions
          achieving B for the first time, it must become true again.
        */

        return any_of(node->necessary_children_begin(),
                      node->greedy_necessary_children_end(),
                      [this] (const LandmarkNode *child) {
            return lm_status[child->get_id()] == FUTURE;});
    }
}

bool StatusManagerLama::landmark_is_leaf(const LandmarkNode &node,
                                         const BitsetView &accepted) const {
    //Note: this is the same as !check_node_orders_disobeyed
    for (const LandmarkNode *parent : node.final_parents) {
        // Note: no condition on edge type here
        if (!accepted.test(parent->get_id())) {
            return false;
        }
    }
    return true;
}
}

