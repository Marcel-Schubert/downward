#include "status_manager_multi_path.h"

using namespace std;

namespace landmarks {
StatusManagerMultiPath::StatusManagerMultiPath(
    LandmarkGraph &graph, bool _add_goal_atoms, bool _add_gn_parents)
    : LandmarkStatusManager(graph), add_goal_atoms(_add_goal_atoms), add_gn_parents(_add_gn_parents) {
    reached_lms.resize(graph.get_num_landmarks());
}

void StatusManagerMultiPath::reset_reached_lms() {
    assert(reached_lms.size() == static_cast<size_t>(
        lm_graph.get_num_landmarks()));
    fill(reached_lms.begin(), reached_lms.end(), -1);
}

void StatusManagerMultiPath::collect_needed_again_relatives(
    const LandmarkNode *node, const State &state) {
    if (add_gn_parents) {
        mark_greedy_necessary_parents_needed_again(node, state);
    }
}

void StatusManagerMultiPath::mark_greedy_necessary_parents_needed_again(
    const LandmarkNode *node, const State &state) {
    /*
      For all A -gn-> B, if B is not accepted and A currently not true,
      since A is a necessary precondition for actions achieving B for
      the first time, it must become true again.
    */

    for (auto it = node->necessary_parents_begin(); it != node->greedy_necessary_parents_end(); ++it) {
        int parent_id = (*it)->get_id();
        if (lm_status[parent_id] == PAST
            && !landmark_is_true_in_state(parent_id, state)) {
            lm_status[parent_id] = PAST_AND_FUTURE;
        }
    }
}

void StatusManagerMultiPath::update_lm_status(
    const State &ancestor_state) {
    const BitsetView accepted = past_lms[ancestor_state];
    int num_landmarks = lm_graph.get_num_landmarks();
    reset_reached_lms();

    /* This first loop is necessary as setup for the *needed again*
       check in the second loop. */
    for (int id = 0; id < num_landmarks; ++id) {
        lm_status[id] = accepted.test(id) ? PAST : FUTURE;
    }
    for (int id = 0; id < num_landmarks; ++id) {
        LandmarkNode *node = lm_graph.get_landmark(id);
        if (lm_status[id] == FUTURE) {
            collect_needed_again_relatives(node, ancestor_state);
        } else if ((lm_status[id] == PAST) && node->is_true_in_goal &&
                   add_goal_atoms && !landmark_is_true_in_state(id, ancestor_state)) {
            lm_status[id] = PAST_AND_FUTURE;
        }
    }
}

void StatusManagerMultiPath::progress_initial_state(
    const State &initial_state) {
    BitsetView accepted = past_lms[initial_state];
    for (auto &node : lm_graph.get_nodes()) {
        if (!node->is_true_in_state(initial_state)) {
            accepted.reset(node->get_id());
        }
    }
}

bool StatusManagerMultiPath::landmark_is_true_in_state(
    int id, const State &state) {
    /* TODO: the use of this function is not as originally intended, because
        its content is deleted both in both update functions. Maybe it is
        obsolete? */
    assert(utils::in_bounds(id, reached_lms));
    if (reached_lms[id] == -1) {
        LandmarkNode *node = lm_graph.get_landmark(id);
        reached_lms[id] = node->is_true_in_state(state) ? 1 : 0;
    }
    assert(reached_lms[id] == 0 || reached_lms[id] == 1);
    return reached_lms[id] == 1;
}

void StatusManagerMultiPath::progress(
    const State &parent_ancestor_state, OperatorID /*op_id*/,
    const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return;
    }

    reset_reached_lms();
    const BitsetView parent_accepted = past_lms[parent_ancestor_state];
    BitsetView accepted = past_lms[ancestor_state];

    int num_landmarks = lm_graph.get_num_landmarks();
    assert(parent_accepted.size() == num_landmarks);
    assert(accepted.size() == num_landmarks);

    for (int id = 0; id < num_landmarks; ++id) {
        if (!parent_accepted.test(id) && accepted.test(id)) {
            if (!landmark_is_true_in_state(id, ancestor_state)) {
                accepted.reset(id);
            }
        }
    }
}

}
