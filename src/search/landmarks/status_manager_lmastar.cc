#include "status_manager_lmastar.h"

namespace landmarks {
StatusManagerLMAstar::StatusManagerLMAstar(LandmarkGraph &lm_graph)
    : LandmarkStatusManager(lm_graph) {
}

void StatusManagerLMAstar::progress_basic(
    const BitsetView &parent_past, const State &parent_ancestor_state,
    BitsetView &past, const State &ancestor_state) {

    utils::unused_variable(parent_ancestor_state);
    for (int id = 0; id < lm_graph.get_num_landmarks(); ++id) {
        if (!parent_past.test(id)) {
            assert(!lm_graph.get_landmark(id)->is_true_in_state(
                parent_ancestor_state));
            /* TODO: Computing whether a landmark is true in a state is
                expensive. It can happen that we compute this multiple times for
                the same state. Maybe we can find a way to circumvent that. */
            if (past.test(id)
                && !lm_graph.get_landmark(id)->is_true_in_state(
                    ancestor_state)) {
                // Found a path where LM_id did not yet hold.
                past.reset(id);
            }
        }
    }
}

void StatusManagerLMAstar::progress_goal(int id, const State &ancestor_state) {
    if (lm_status[id] == PAST) {
        LandmarkNode *lm = lm_graph.get_landmark(id);
        if (lm->is_true_in_goal && !lm->is_true_in_state(ancestor_state)) {
            lm_status[id] = PAST_AND_FUTURE;
        }
    }
}

void StatusManagerLMAstar::progress_greedy_necessary(
    int id, const State &ancestor_state) {
    /*
      For all A -gn-> B, if B is not past and A currently not true,
      since A is a necessary precondition for actions achieving B for
      the first time, it must become true again.
    */
    if (lm_status[id] == FUTURE) {
        LandmarkNode *lm = lm_graph.get_landmark(id);
        for (auto it = lm->necessary_parents_begin();
             it != lm->greedy_necessary_parents_end(); ++it) {
            int parent_id = (*it)->get_id();
            if (lm_status[parent_id] == PAST
                && !(*it)->is_true_in_state(ancestor_state)) {
                lm_status[parent_id] = PAST_AND_FUTURE;
            }
        }
    }
}

void StatusManagerLMAstar::update_lm_status(const State &ancestor_state) {
    const BitsetView past = past_lms[ancestor_state];
    int num_landmarks = lm_graph.get_num_landmarks();

    // Basic progression (again, translating to landmark status)
    for (int id = 0; id < num_landmarks; ++id) {
        lm_status[id] = past.test(id) ? PAST : FUTURE;
    }

    for (int id = 0; id < num_landmarks; ++id) {
        progress_goal(id, ancestor_state);
        progress_greedy_necessary(id, ancestor_state);
    }
}

void StatusManagerLMAstar::progress_initial_state(const State &initial_state) {
    BitsetView past = past_lms[initial_state];
    for (auto &node : lm_graph.get_nodes()) {
        if (!node->is_true_in_state(initial_state)) {
            past.reset(node->get_id());
        }
    }
}

void StatusManagerLMAstar::progress(const State &parent_ancestor_state,
                                    OperatorID /*op_id*/,
                                    const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return;
    }

    const BitsetView parent_past = past_lms[parent_ancestor_state];
    BitsetView past = past_lms[ancestor_state];

    int num_landmarks = lm_graph.get_num_landmarks();
    assert(parent_past.size() == num_landmarks);
    assert(past.size() == num_landmarks);
    utils::unused_variable(num_landmarks);

    progress_basic(parent_past, parent_ancestor_state, past, ancestor_state);
}
}
