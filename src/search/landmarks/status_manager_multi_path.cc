#include "status_manager_multi_path.h"

#include "landmark.h"

#include "../utils/logging.h"

using namespace std;

namespace landmarks {
StatusManagerMultiPath::StatusManagerMultiPath(
    LandmarkGraph &graph, bool add_goal_atoms, bool add_gn_parents,
    bool add_reasonable_children)
    : LandmarkStatusManager(graph),
      add_goal_atoms(add_goal_atoms),
      add_gn_parents(add_gn_parents),
      add_reasonalbe_children(add_reasonable_children) {
    reached_lms.resize(graph.get_num_landmarks());
}

void StatusManagerMultiPath::reset_reached_lms() {
    assert(reached_lms.size() == static_cast<size_t>(
               lm_graph.get_num_landmarks()));
    fill(reached_lms.begin(), reached_lms.end(), -1);
}

void StatusManagerMultiPath::collect_needed_again_relatives(
    const LandmarkNode *node, const State &state) {
    assert(lm_status[node->get_id()] == REQUIRED);
    if (add_gn_parents) {
        mark_greedy_necessary_parents_needed_again(node, state);
    }
    if (add_reasonalbe_children) {
        mark_reasonable_children_needed_again(node);
    }
}

void StatusManagerMultiPath::mark_greedy_necessary_parents_needed_again(
    const LandmarkNode *node, const State &state) {
    /*
      For all A -gn-> B, if B is not accepted and A currently not true,
      since A is a necessary precondition for actions achieving B for
      the first time, it must become true again.
    */
    for (auto &parent : node->parents) {
        if (parent.second >= EdgeType::GREEDY_NECESSARY) {
            int parent_id = parent.first->get_id();
            if (lm_status[parent_id] == ACCEPTED
                && !parent.first->get_landmark().is_true_in_state(state)) {
                lm_status[parent_id] = ACCEPTED_AND_REQUIRED;
            }
        }
    }
}

void StatusManagerMultiPath::mark_reasonable_children_needed_again(
    const LandmarkNode *node) {
    /*
      For all A -r-> B where A is not first added but B is, B must
      be destroyed to achieve A (definition of reasonable
      orderings). Hence, B is needed again.
    */
    for (auto &edge : node->children) {
        const LandmarkNode *child = edge.first;
        if (edge.second == EdgeType::REASONABLE) {
            int child_id = child->get_id();
            if (lm_status[child_id] == ACCEPTED) {
                lm_status[child_id] = ACCEPTED_AND_REQUIRED;
            }
        }
    }
}

void StatusManagerMultiPath::update_lm_status(
    const State &ancestor_state) {
    const BitsetView accepted = accepted_lms[ancestor_state];
    int num_landmarks = lm_graph.get_num_landmarks();
    reset_reached_lms();

    /* This first loop is necessary as setup for the *needed again*
       check in the second loop. */
    for (int id = 0; id < num_landmarks; ++id) {
        lm_status[id] = accepted.test(id) ? ACCEPTED : REQUIRED;
    }
    for (int id = 0; id < num_landmarks; ++id) {
        LandmarkNode *node = lm_graph.get_node(id);
        if (lm_status[id] == REQUIRED) {
            collect_needed_again_relatives(node, ancestor_state);
        } else if (lm_status[id] == ACCEPTED) {
            Landmark &landmark = node->get_landmark();
            if (landmark.is_true_in_goal && add_goal_atoms
                && !landmark.is_true_in_state(ancestor_state)) {
                lm_status[id] = ACCEPTED_AND_REQUIRED;
            }
        }
    }
}

void StatusManagerMultiPath::set_landmarks_for_initial_state(
    const State &initial_state) {
    BitsetView accepted = accepted_lms[initial_state];
    for (auto &node : lm_graph.get_nodes()) {
        if (!node->get_landmark().is_true_in_state(initial_state)) {
            accepted.reset(node->get_id());
        }
    }
}

bool StatusManagerMultiPath::update_accepted_landmarks(
    const State &parent_ancestor_state, OperatorID /*op_id*/,
    const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return false;
    }

    const BitsetView parent_accepted = accepted_lms[parent_ancestor_state];
    BitsetView accepted = accepted_lms[ancestor_state];

    int num_landmarks = lm_graph.get_num_landmarks();
    assert(parent_accepted.size() == num_landmarks);
    assert(accepted.size() == num_landmarks);

    /*
      Consider all landmarks that are not accepted by this parent. If they are
      accepted in the current state (either because they hold in this state or
      they were accepted on all previously considered paths), they are not
      accepted anymore (unless they hold in this state).

      Over multiple paths, this has the effect of computing the intersection of
      "accepted" for all parents. It is important here that upon first visit,
      all elements in "accepted" are true because true is the neutral element of
      intersection.
    */
    for (int id = 0; id < num_landmarks; ++id) {
        if (!parent_accepted.test(id) && accepted.test(id)) {
            /* TODO: It may be very inefficient to check this for all landmarks
                separately (and potentially multiple times?). */
            if (!lm_graph.get_node(id)->get_landmark().is_true_in_state(
                    ancestor_state)) {
                accepted.reset(id);
            }
        }
    }
    return true;
}
}
