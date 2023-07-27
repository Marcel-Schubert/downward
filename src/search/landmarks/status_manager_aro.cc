#include "status_manager_aro.h"

using namespace std;

namespace landmarks {
StatusManagerARO::StatusManagerARO(LandmarkGraph &lm_graph)
    : LandmarkStatusManager(lm_graph),
      future_lms(vector<bool>(lm_graph.get_num_landmarks(), false)) {
}

void StatusManagerARO::progress_basic(
    const BitsetView &parent_past, const BitsetView &parent_fut,
    const State &parent_ancestor_state, BitsetView &past,
    BitsetView &fut, const State &ancestor_state) {
    utils::unused_variable(parent_fut);
    utils::unused_variable(parent_ancestor_state);
    for (int id = 0; id < lm_graph.get_num_landmarks(); ++id) {
        if (parent_fut.test(id)) {
            /* TODO: Computing whether a landmark is true in a state is
                expensive. It can happen that we compute this multiple times for
                the same state. Maybe we can find a way to circumvent that. */
            if (!lm_graph.get_landmark(id)->is_true_in_state(ancestor_state)) {
                fut.set(id);
                if (!parent_past.test(id)) {
                    past.reset(id);
                }
            }
        }
    }
}

void StatusManagerARO::progress_goal(
    int id, const State &ancestor_state, BitsetView &fut) {
    if (!fut.test(id)) {
        LandmarkNode *lm = lm_graph.get_landmark(id);
        if (lm->is_true_in_goal && !lm->is_true_in_state(ancestor_state)) {
            fut.set(id);
        }
    }
}

void StatusManagerARO::progress_greedy_necessary(
    int id, const State &ancestor_state, const BitsetView &past,
    BitsetView &fut) {
    /*
      For all A -gn-> B, if B is not past and A currently not true,
      since A is a necessary precondition for actions achieving B for
      the first time, it must become true again.
    */
    if (!past.test(id)) {
        LandmarkNode *lm = lm_graph.get_landmark(id);
        for (auto it = lm->necessary_parents_begin();
             it != lm->greedy_necessary_parents_end(); ++it) {
            int parent_id = (*it)->get_id();
            if (!fut.test(parent_id)
                && !(*it)->is_true_in_state(ancestor_state)) {
                fut.set(parent_id);
            }
        }
    }
}

void StatusManagerARO::progress_reasonable(
    int id, const BitsetView &past, BitsetView &fut) {
    if (!past.test(id)) {
        LandmarkNode *lm = lm_graph.get_landmark(id);
        for (auto it = lm->reasonable_children_begin();
             it != lm->reasonable_children_end(); ++it) {
            fut.set((*it)->get_id());
        }
    }
}

void StatusManagerARO::update_lm_status(const State &ancestor_state) {
    const BitsetView past = past_lms[ancestor_state];
    const BitsetView fut = future_lms[ancestor_state];

    const int num_landmarks = lm_graph.get_num_landmarks();
    for (int id = 0; id < num_landmarks; ++id) {
        if (!past.test(id)) {
            assert(fut.test(id));
            lm_status[id] = FUTURE;
        } else if (!fut.test(id)) {
            assert(past.test(id));
            lm_status[id] = PAST;
        } else {
            lm_status[id] = PAST_AND_FUTURE;
        }
    }
}

void StatusManagerARO::progress_initial_state(const State &initial_state) {
    BitsetView past = past_lms[initial_state];
    BitsetView future = future_lms[initial_state];
    for (auto &node : lm_graph.get_nodes()) {
        int id = node->get_id();
        if (node->is_true_in_state(initial_state)) {
            for (auto &parent : node->parents) {
                utils::unused_variable(parent);
                assert(parent.second == EdgeType::REASONABLE);
                assert(!parent.first->is_true_in_state(initial_state));
                future.set(id);
            }
        } else {
            past.reset(id);
            future.set(id);
        }
    }
}

void StatusManagerARO::progress(const State &parent_ancestor_state,
                                OperatorID /*op_id*/, const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return;
    }

    const BitsetView parent_past = past_lms[parent_ancestor_state];
    BitsetView past = past_lms[ancestor_state];

    const BitsetView parent_fut = future_lms[parent_ancestor_state];
    BitsetView fut = future_lms[ancestor_state];

    int num_landmarks = lm_graph.get_num_landmarks();
    assert(past.size() == num_landmarks);
    assert(parent_past.size() == num_landmarks);
    assert(fut.size() == num_landmarks);
    assert(parent_fut.size() == num_landmarks);

    progress_basic(parent_past, parent_fut, parent_ancestor_state, past,
                   fut, ancestor_state);
    for (int id = 0; id < num_landmarks; ++id) {
        progress_goal(id, ancestor_state, fut);
        progress_greedy_necessary(id, ancestor_state, past, fut);
        progress_reasonable(id, past, fut);
    }
}

}
