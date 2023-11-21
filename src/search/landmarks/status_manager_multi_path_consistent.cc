#include "status_manager_multi_path_consistent.h"

#include "landmark.h"

#include "../utils/logging.h"

using namespace std;

namespace landmarks {
StatusManagerMultiPathConsistent::StatusManagerMultiPathConsistent(
    LandmarkGraph &graph, bool add_goal_atoms, bool add_gn_parents,
    bool add_reasonable_children)
    : LandmarkStatusManager(graph),
      add_goal_atoms(add_goal_atoms),
      add_gn_parents(add_gn_parents),
      add_reasonalbe_children(add_reasonable_children),
      required_lms(vector<bool>(graph.get_num_landmarks(), false)) {
}

void StatusManagerMultiPathConsistent::update_lm_status(
    const State &ancestor_state) {
    const BitsetView accepted = accepted_lms[ancestor_state];
    const BitsetView required = required_lms[ancestor_state];
    int num_landmarks = lm_graph.get_num_landmarks();

    for (int id = 0; id < num_landmarks; ++id) {
        if (accepted.test(id)) {
            if (required.test(id)) {
                lm_status[id] = ACCEPTED_AND_REQUIRED;
            } else {
                lm_status[id] = ACCEPTED;
            }
        } else {
            assert(required.test(id));
            lm_status[id] = REQUIRED;
        }
    }
}

void StatusManagerMultiPathConsistent::set_landmarks_for_initial_state(
    const State &initial_state) {
    BitsetView accepted = accepted_lms[initial_state];
    BitsetView required = required_lms[initial_state];
    for (auto &node : lm_graph.get_nodes()) {
        if (!node->get_landmark().is_true_in_state(initial_state)) {
            int id = node->get_id();
            accepted.reset(id);
            required.set(id);
        }
    }
    for (auto &node : lm_graph.get_nodes()) {
        int id = node->get_id();
        if (!accepted.test(id)) {
            mark_required_again_relatives(node.get(), initial_state, required);
        }
    }
}

void StatusManagerMultiPathConsistent::mark_required_again_relatives(
    const LandmarkNode *node, const State &state,
    BitsetView &required_landmarks) {
    if (add_gn_parents) {
        set_greedy_necessary_parents_required(node, state, required_landmarks);
    }
    if (add_reasonalbe_children) {
        set_reasonable_children_required(node, required_landmarks);
    }
}

void StatusManagerMultiPathConsistent::set_greedy_necessary_parents_required(
    const LandmarkNode *node, const State &state, BitsetView &required) {
    /*
      For all A -gn-> B, if B is not accepted and A currently not true,
      since A is a necessary precondition for actions achieving B for
      the first time, it must become true again.
    */
    for (auto &parent : node->parents) {
        if (parent.second >= EdgeType::GREEDY_NECESSARY) {
            if (!parent.first->get_landmark().is_true_in_state(state)) {
                required.set(parent.first->get_id());
            }
        }
    }
}

void StatusManagerMultiPathConsistent::set_reasonable_children_required(
    const LandmarkNode *node, BitsetView &required) {
    /*
      For all A -r-> B where A is not first added but B is, B must
      be destroyed to achieve A (definition of reasonable
      orderings). Hence, B is needed again.
    */
    for (auto &child : node->children) {
        if (child.second == EdgeType::REASONABLE) {
            required.set(child.first->get_id());
        }
    }
}

bool StatusManagerMultiPathConsistent::update_accepted_landmarks(
    const State &parent_ancestor_state, OperatorID /*op_id*/,
    const State &ancestor_state) {
    if (ancestor_state == parent_ancestor_state) {
        // This can happen, e.g., in Satellite-01.
        return false;
    }

    BitsetView accepted = accepted_lms[ancestor_state];
    BitsetView required = required_lms[ancestor_state];
    int num_landmarks = lm_graph.get_num_landmarks();
    assert(accepted.size() == num_landmarks);
    assert(required.size() == num_landmarks);

    int num_blocks = BitsetMath::compute_num_blocks(num_landmarks);

    vector<BitsetMath::Block> packed_accepted =
        accepted_lms.get_duplicate(parent_ancestor_state);
    BitsetView accepted_update(ArrayView<BitsetMath::Block>(
        packed_accepted.data(), num_blocks), num_landmarks);

    vector<BitsetMath::Block> packed_required =
        required_lms.get_duplicate(parent_ancestor_state);
    BitsetView required_update(ArrayView<BitsetMath::Block>(
        packed_required.data(), num_blocks), num_landmarks);

    /* This first loop is necessary as setup for the
       *mark_required_again* check in the second loop. */
    for (int id = 0; id < num_landmarks; ++id) {
        Landmark &landmark = lm_graph.get_node(id)->get_landmark();
        if (required_update.test(id)
            && landmark.is_true_in_state(ancestor_state)) {
            accepted_update.set(id);
            // Test whether landmark was "added" in the last step.
            if (!landmark.is_true_in_state(parent_ancestor_state)) {
                required_update.reset(id);
            }
        }
    }

    for (int id = 0; id < num_landmarks; ++id) {
        LandmarkNode *node = lm_graph.get_node(id);
        if (!accepted_update.test(id)) {
            mark_required_again_relatives(
                node, ancestor_state,required_update);
        } else if (!required_update.test(id)) {
            Landmark &landmark = node->get_landmark();
            if (landmark.is_true_in_goal && add_goal_atoms
                && !landmark.is_true_in_state(ancestor_state)) {
                required_update.set(id);
            }
        }
    }

    accepted.intersect(accepted_update);
    required.unite(required_update);
    return true;
}

}
