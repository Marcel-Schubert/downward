#include "landmark_status_manager.h"

#include "../utils/logging.h"

using namespace std;

namespace landmarks {
/*
  By default we mark all landmarks as reached, since we do an intersection when
  computing new landmark information.
*/
LandmarkStatusManager::LandmarkStatusManager(LandmarkGraph &graph)
    : past_lms(vector<bool>(graph.get_num_landmarks(), true)),
      lm_status(graph.get_num_landmarks(), FUTURE),
      lm_graph(graph) {
}

unordered_set<const LandmarkNode *>LandmarkStatusManager::get_past_landmarks(
    const State &state) {
    BitsetView accepted = past_lms[state];
    LandmarkSet landmark_set;
    for (int i = 0; i < accepted.size(); ++i)
        if (accepted.test(i))
            landmark_set.insert(lm_graph.get_landmark(i));
    return landmark_set;
}

bool LandmarkStatusManager::dead_end_exists() {
    for (auto &node : lm_graph.get_nodes()) {
        int id = node->get_id();

        /*
          This dead-end detection works for the following case:
          X is a goal, it is true in the initial state, and has no achievers.
          Some action A has X as a delete effect. Then using this,
          we can detect that applying A leads to a dead-end.

          Note: this only tests for reachability of the landmark from the initial state.
          A (possibly) more effective option would be to test reachability of the landmark
          from the current state.
        */

        if (!node->is_derived) {
            if (!landmark_is_past(id) &&
                node->first_achievers.empty()) {
                return true;
            }
            if (landmark_is_future(id) &&
                node->possible_achievers.empty()) {
                return true;
            }
        }
    }
    return false;
}
}
