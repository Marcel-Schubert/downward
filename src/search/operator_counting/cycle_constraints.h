#ifndef CYCLIC_LANDMARKS_CYCLE_CONSTRAINTS_H
#define CYCLIC_LANDMARKS_CYCLE_CONSTRAINTS_H

#include "../algorithms/named_vector.h"
#include "../lp/lp_solver.h"
#include "constraint_generator.h"

#include <map>

namespace options {
class Options;
}

namespace landmarks {
class DisjActionLandmarkGraph;
class LandmarkFactory;
class LandmarkGraph;
}

using weighted_adj_list = std::vector<std::map<int, float>>;
using adj_list = std::vector<std::vector<int>>;
using dalm_graph = std::shared_ptr<landmarks::DisjActionLandmarkGraph>;

namespace operator_counting {
enum class CycleGenerator {
    NONE,
    johnson,
    floyd_warshall,
    depth_first,
};

void add_cycle_constraint_option_to_parser(options::OptionParser &parser);

class CycleConstraints : public ConstraintGenerator {
    const options::Options &opts;
    const CycleGenerator cycle_generator;
    const bool strong;
    bool first_call = true;

    void add_constraints_for_all_cycles(const dalm_graph &lm_graph,
                                        lp::LPSolver &lp_solver);
    void add_constraints_implicit_hitting_set_approach(
        const dalm_graph &lm_graph, lp::LPSolver &lp_solver);
    static adj_list compute_adj_list(const dalm_graph &lm_graph);
    static std::vector<float> compute_landmark_weights(
        const dalm_graph &lm_graph, const std::vector<double> &counts);
    lp::LPConstraint compute_constraint(const dalm_graph &lm_graph,
                                        const std::vector<int> &cycle,
                                        double infinity);
    std::map<int, size_t> count_lm_occurrences(
        const dalm_graph &lm_graph, const std::vector<int> &cycle) const;
public:
    explicit CycleConstraints(const options::Options &opts);
    virtual void initialize_constraints(
        const std::shared_ptr<AbstractTask> &task,
        named_vector::NamedVector<lp::LPConstraint> &constraints,
        double infinity) override;
    virtual bool update_constraints(const State &state,
                                    lp::LPSolver &lp_solver) override;
};
}

#endif
