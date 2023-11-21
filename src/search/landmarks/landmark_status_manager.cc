#include "landmark_status_manager.h"

#include "landmark.h"

#include "../option_parser.h"
#include "../utils/logging.h"

using namespace std;

namespace landmarks {
/*
  By default we mark all landmarks as reached, since we do an intersection when
  computing new landmark information.
*/
LandmarkStatusManager::LandmarkStatusManager(LandmarkGraph &graph)
    : accepted_lms(vector<bool>(graph.get_num_landmarks(), true)),
      lm_status(graph.get_num_landmarks(), REQUIRED),
      lm_graph(graph) {
}

BitsetView LandmarkStatusManager::get_accepted_landmarks(const State &state) {
    return accepted_lms[state];
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

        const Landmark &landmark = node->get_landmark();
        if (!landmark.is_derived) {
            if ((lm_status[id] == REQUIRED) &&
                landmark.first_achievers.empty()) {
                return true;
            }
            if ((lm_status[id] == ACCEPTED_AND_REQUIRED) &&
                landmark.possible_achievers.empty()) {
                return true;
            }
        }
    }
    return false;
}

void add_status_manager_options_to_parser(OptionParser &parser) {
    vector<string> landmark_status_manager_types;
    landmark_status_manager_types.push_back("LAMA");
    landmark_status_manager_types.push_back("MULTI_PATH");
    landmark_status_manager_types.push_back("CONSISTENT");
    parser.add_enum_option<LandmarkStatusManagerType>(
        "status_manager", landmark_status_manager_types,
        "choose status manager type", "MULTI_PATH");

    parser.add_option<bool>(
        "add_goal_atoms", "Mark goal atoms as required again.",
        "true");
    parser.add_option<bool>(
        "add_gn_parents",
        "Mark greedy-necessary parents as required again.", "true");
    parser.add_option<bool>(
        "add_reasonable_children",
        "Mark reasonable children as required again.", "true");
}
}
