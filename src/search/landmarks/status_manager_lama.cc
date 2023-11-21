#include "status_manager_lama.h"

#include "landmark.h"

#include "../utils/logging.h"

using namespace std;

namespace landmarks {
StatusManagerLama::StatusManagerLama(LandmarkGraph &lm_graph)
    : LandmarkStatusManager(lm_graph) {
}

void StatusManagerLama::set_landmarks_for_initial_state(
    const State &initial_state) {
    BitsetView accepted = get_accepted_landmarks(initial_state);

    for (auto &node : lm_graph.get_nodes()) {
        if (!node->get_landmark().is_true_in_state(initial_state)
            || !node->parents.empty()) {
            accepted.reset(node->get_id());
        }
    }
}

bool StatusManagerLama::update_accepted_landmarks(
    const State &parent_ancestor_state, OperatorID /*op_id*/,
    const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return false;
    }

    const BitsetView parent_accepted = get_accepted_landmarks(
        parent_ancestor_state);
    BitsetView accepted = get_accepted_landmarks(ancestor_state);

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
            LandmarkNode *node = lm_graph.get_node(id);
            if (node->get_landmark().is_true_in_state(ancestor_state)) {
                if (landmark_is_leaf(*node, accepted)) {
                    accepted.set(id);
                }
            }
        }
    }

    return true;
}

void StatusManagerLama::update_lm_status(const State &ancestor_state) {
    const BitsetView accepted = get_accepted_landmarks(ancestor_state);

    const int num_landmarks = lm_graph.get_num_landmarks();
    /* This first loop is necessary as setup for the *needed again*
       check in the second loop. */
    for (int id = 0; id < num_landmarks; ++id) {
        lm_status[id] = accepted.test(id) ? ACCEPTED : REQUIRED;
    }
    for (int id = 0; id < num_landmarks; ++id) {
        if (lm_status[id] == ACCEPTED
            && landmark_needed_again(id, ancestor_state)) {
            lm_status[id] = ACCEPTED_AND_REQUIRED;
        }
    }
}

bool StatusManagerLama::landmark_needed_again(
    int id, const State &state) {
    LandmarkNode *node = lm_graph.get_node(id);
    const Landmark &landmark = node->get_landmark();
    if (landmark.is_true_in_state(state)) {
        return false;
    } else if (landmark.is_true_in_goal) {
        return true;
    } else {
        /*
          For all A ->_gn B, if B is not accepted and A currently not
          true, since A is a necessary precondition for actions
          achieving B for the first time, it must become true again.
        */
        for (const auto &child : node->children) {
            if (child.second >= EdgeType::GREEDY_NECESSARY
                && lm_status[child.first->get_id()] == REQUIRED) {
                return true;
            }
        }
        return false;
    }
}

bool StatusManagerLama::landmark_is_leaf(
    const LandmarkNode &node, const BitsetView &accepted) const {
    //Note: this is the same as !check_node_orders_disobeyed
    for (const auto &parent : node.parents) {
        LandmarkNode *parent_node = parent.first;
        // Note: no condition on edge type here
        if (!accepted.test(parent_node->get_id())) {
            return false;
        }
    }
    return true;
}
}
