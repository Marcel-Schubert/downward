#ifndef LANDMARKS_LANDMARK_STATUS_MANAGER_H
#define LANDMARKS_LANDMARK_STATUS_MANAGER_H

#include "landmark_graph.h"

#include "../per_state_bitset.h"

namespace landmarks {
class LandmarkGraph;
class LandmarkNode;

enum LandmarkStatus {
    ACCEPTED = 0, REQUIRED = 1, ACCEPTED_AND_REQUIRED = 2,
};

enum LandmarkStatusManagerType {
    LAMA, MULTI_PATH, CONSISTENT,
};

void add_status_manager_options_to_parser(options::OptionParser &parser);

class LandmarkStatusManager {
protected:
    PerStateBitset accepted_lms;
    std::vector<LandmarkStatus> lm_status;

    LandmarkGraph &lm_graph;
public:
    explicit LandmarkStatusManager(LandmarkGraph &graph);
    virtual ~LandmarkStatusManager() = default;

    BitsetView get_accepted_landmarks(const State &state);

    virtual void update_lm_status(const State &ancestor_state) = 0;
    bool dead_end_exists();

    virtual void set_landmarks_for_initial_state(
        const State &initial_state) = 0;
    virtual bool update_accepted_landmarks(
        const State &parent_ancestor_state, OperatorID op_id,
        const State &ancestor_state) = 0;

    /*
      TODO:
      The status of a landmark is actually dependent on the state. This
      is not represented in the function below. Furthermore, the status
      manager only stores the status for one particular state at a time.

      At the day of writing this comment, this works as
      *update_accepted_landmarks()* is always called before the status
      information is used (by calling *get_landmark_status()*).

      It would be a good idea to ensure that the status for the
      desired state is returned at all times, or an error is thrown
      if the desired information does not exist.
     */
    LandmarkStatus get_landmark_status(size_t id) const {
        assert(static_cast<int>(id) < lm_graph.get_num_landmarks());
        return lm_status[id];
    }
};
}

#endif
