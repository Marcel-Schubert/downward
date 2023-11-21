#ifndef CYCLIC_LANDMARKS_DISJ_ACTION_LANDMARK_GRAPH_H
#define CYCLIC_LANDMARKS_DISJ_ACTION_LANDMARK_GRAPH_H

#include "../landmarks/landmark_graph.h"
#include "../landmarks/landmark_status_manager.h"

#include <map>
#include <set>
#include <vector>

namespace landmarks {
enum class OrderingType {
    strong,
    weak,
};

class DisjActionLandmarkNode {
    std::map<int, OrderingType> dependencies;

public:
    const std::set<int> actions;

    DisjActionLandmarkNode(std::set<int> actions);
    bool overlaps_with(DisjActionLandmarkNode &other) const;
    bool satisfied_by(int op_id) const;
    void add_strong_dependency(int node_id, size_t &num_orderings, size_t &num_weak_orderings);
    void add_weak_dependency(int node_id, size_t &num_orderings, size_t &num_weak_orderings);
    OrderingType get_dependency(int id) const;
    const std::map<int, OrderingType> &get_dependencies() const;
    bool depends_on(int id) const;
};

class DisjActionLandmarkGraph {
    std::map<std::set<int>, size_t> ids;
    std::vector<DisjActionLandmarkNode> lms;
    size_t num_orderings = 0;
    size_t num_weak_orderings = 0;

    std::vector<size_t> add_nodes(const LandmarkGraph &lm_graph,
                                  const State &init_state);
    void add_edges(const LandmarkGraph &lm_graph,
                   const State &init_state,
                   const std::vector<size_t> &fact_to_action_lm_map);

    std::vector<size_t> add_required_nodes(
        const LandmarkGraph &lmgraph, const LandmarkStatusManager &manager);
    void add_required_edges(
        const LandmarkGraph &lm_graph, const LandmarkStatusManager &manager,
        const State &state, const std::vector<size_t> &fact_to_action_lm_map);

    size_t add_node(const std::set<int> &actions);
    void add_edge(size_t from_id, size_t to_id, EdgeType edge_type);

    void dump_lm(int id) const;
public:
    explicit DisjActionLandmarkGraph(const LandmarkGraph &lm_graph,
                                     const State &init_state);
    explicit DisjActionLandmarkGraph(
        const LandmarkGraph &lm_graph,
        const LandmarkStatusManager &manager, const State &state);
    size_t get_number_of_landmarks() const;
    size_t get_number_of_orderings() const {
        return num_orderings;
    }
    size_t get_number_of_weak_orderings() const {
        return num_weak_orderings;
    }
    size_t get_number_of_strong_orderings() const {
        return num_orderings - num_weak_orderings;
    }
    const std::set<int> &get_actions(int id);
    const std::map<int, OrderingType> &get_dependencies(int id);
    OrderingType get_ordering_type(int from, int to);
    void dump() const;
    void dump_dot() const;

    std::vector<std::map<int, bool>> to_adj_list() const;
};
}

#endif
