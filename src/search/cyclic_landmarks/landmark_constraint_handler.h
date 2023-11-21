#ifndef CYCLIC_LANDMARKS_LANDMARK_CONSTRAINT_HANDLER_H
#define CYCLIC_LANDMARKS_LANDMARK_CONSTRAINT_HANDLER_H

#include "../landmarks/landmark_status_manager.h"
#include "../option_parser.h"
#include "../task_proxy.h"

namespace landmarks {
class DisjActionLandmarkGraph;
class LandmarkFactory;
class LandmarkGraph;
class LandmarkStatusManager;

class LandmarkConstraintHandler {
    LandmarkConstraintHandler() = default;
    ~LandmarkConstraintHandler() = default;

    long num_landmarks = 0;
    long num_orderings = 0;
    long num_weak_orderings = 0;
    int num_lm_graphs = 0;

    bool initialized = false;
    std::shared_ptr<LandmarkFactory> lm;
    std::shared_ptr<AbstractTask> task;
    std::shared_ptr<TaskProxy> task_proxy;
    bool path_dependent;
    std::shared_ptr<LandmarkGraph> lm_graph;
    LandmarkStatusManagerType lm_status_manager_type;
    bool add_goal_atoms;
    bool add_gn_parents;
    bool add_reasonable_children;
    std::shared_ptr<LandmarkStatusManager> lm_status_manager;

    std::shared_ptr<DisjActionLandmarkGraph> dalm_graph;
    StateID last_state_id = StateID::no_state;

    void compute_lm_graph(const State &state);

    void compute_initial_lm_graph(const State &state);

    void recompute_lm_graph(const State &state);
public:
    static long num_cycles;

    static LandmarkConstraintHandler &get_instance();
    void initialize(const options::Options &opts,
                    const std::shared_ptr<AbstractTask> &task);
    std::shared_ptr<DisjActionLandmarkGraph> get_lm_graph(const State &state);

    void print_statistics() const;

    LandmarkStatusManager &get_lm_status_manager() {
        assert(path_dependent);
        assert(lm_status_manager);
        return *lm_status_manager;
    }

    bool initial_fact_landmark_graph_is_acyclic();
};
}

#endif
