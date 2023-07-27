#ifndef LANDMARKS_LANDMARK_STATUS_MANAGER_H
#define LANDMARKS_LANDMARK_STATUS_MANAGER_H

#include "landmark_graph.h"

#include "../per_state_bitset.h"

namespace landmarks {
class LandmarkGraph;
class LandmarkNode;

enum landmark_status {PAST = 0, FUTURE = 1, PAST_AND_FUTURE = 2};

class LandmarkStatusManager {
protected:
    PerStateBitset past_lms;
    std::vector<landmark_status> lm_status;
    LandmarkGraph &lm_graph;
public:
    std::unordered_set<const LandmarkNode *> get_past_landmarks(
        const State &state);

    explicit LandmarkStatusManager(LandmarkGraph &graph);
    virtual ~LandmarkStatusManager() = default;

    virtual void update_lm_status(const State &ancestor_state) = 0;
    bool dead_end_exists();

    virtual void progress_initial_state(const State &initial_state) = 0;
    virtual void progress(const State &parent_ancestor_state,
                          OperatorID op_id, const State &ancestor_state) = 0;

    /*
      TODO:
      The status of a landmark is actually dependent on the state. This
      is not represented in the functions below. Furthermore, the status
      manager only stores the status for one particular state at a time.

      At the day of writing this comment, this works as
      *update_accepted_lms()* is always called before the status
      information is used (by calling *get_landmark_status()*).

      It would be a good idea to ensure that the status for the
      desired state is returned at all times, or an error is thrown
      if the desired information does not exist.
     */
    bool landmark_is_past(int id) {
        return lm_status[id] != FUTURE;
    };
    bool landmark_is_future(int id) {
        return lm_status[id] != PAST;
    };
};
}

#endif
