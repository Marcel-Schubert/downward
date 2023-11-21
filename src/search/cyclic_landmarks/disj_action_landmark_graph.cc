#include "disj_action_landmark_graph.h"

#include <cassert>

using namespace std;

namespace landmarks {
DisjActionLandmarkNode::DisjActionLandmarkNode(std::set<int> actions)
    : actions(move(actions)) {
}

bool DisjActionLandmarkNode::overlaps_with(
    DisjActionLandmarkNode &other) const {
    vector<int> intersect;
    set_intersection(actions.begin(), actions.end(),
                     other.actions.begin(), other.actions.end(),
                     back_inserter(intersect));
    return !intersect.empty();
}

bool DisjActionLandmarkNode::satisfied_by(int op_id) const {
    return actions.find(op_id) != actions.end();
}

void DisjActionLandmarkNode::add_strong_dependency(
    int node_id, size_t &num_orderings, size_t &num_weak_orderings) {
    if (dependencies.find(node_id) != dependencies.end()) {
        if (dependencies[node_id] == OrderingType::weak) {
            --num_weak_orderings;
        }
        dependencies[node_id] = OrderingType::strong;
    } else {
        dependencies[node_id] = OrderingType::strong;
        ++num_orderings;
    }
}

void DisjActionLandmarkNode::add_weak_dependency(
    int node_id, size_t &num_orderings, size_t &num_weak_orderings) {
    if (dependencies.find(node_id) == dependencies.end()) {
        dependencies[node_id] = OrderingType::weak;
        ++num_weak_orderings;
        ++num_orderings;
    }
}

OrderingType DisjActionLandmarkNode::get_dependency(int node_id) const {
    assert(depends_on(node_id));
    return dependencies.at(node_id);
}

const map<int, OrderingType> &DisjActionLandmarkNode::get_dependencies() const {
    return dependencies;
}

bool DisjActionLandmarkNode::depends_on(int node_id) const {
    return dependencies.count(node_id);
}

DisjActionLandmarkGraph::DisjActionLandmarkGraph(
    const LandmarkGraph &lm_graph, const State &init_state) {
    const vector<size_t> fact_to_action_lm_map =
        add_nodes(lm_graph, init_state);
    add_edges(lm_graph, init_state, fact_to_action_lm_map);
}

DisjActionLandmarkGraph::DisjActionLandmarkGraph(
    const LandmarkGraph &lm_graph, const LandmarkStatusManager &manager,
    const State &state) {
    const vector<size_t> fact_to_action_lm_map =
        add_required_nodes(lm_graph, manager);
    add_required_edges(lm_graph, manager, state, fact_to_action_lm_map);
}

vector<size_t> DisjActionLandmarkGraph::add_nodes(
    const LandmarkGraph &lm_graph, const State &init_state) {
    vector<size_t> fact_to_action_lm_map(lm_graph.get_num_landmarks(), -1);
    for (const unique_ptr<LandmarkNode> &node : lm_graph.get_nodes()) {
        if (!node->get_landmark().is_true_in_state(init_state)
            || !node->parents.empty()) {
            fact_to_action_lm_map[node->get_id()] =
                add_node(node->get_landmark().possible_achievers);
        }
    }
    return fact_to_action_lm_map;
}

vector<size_t> DisjActionLandmarkGraph::add_required_nodes(
    const LandmarkGraph &lm_graph, const LandmarkStatusManager &manager) {
    vector<size_t> fact_to_action_lm_map(lm_graph.get_num_landmarks(), -1);
    for (const unique_ptr<LandmarkNode> &node : lm_graph.get_nodes()) {
        int id = node->get_id();
        if (manager.get_landmark_status(id) != LandmarkStatus::ACCEPTED) {
            fact_to_action_lm_map[id] =
                add_node(node->get_landmark().possible_achievers);
        }
    }
    return fact_to_action_lm_map;
}

void DisjActionLandmarkGraph::add_edges(
    const LandmarkGraph &lm_graph, const State &init_state,
    const vector<size_t> &fact_to_action_lm_map) {
    for (auto &node : lm_graph.get_nodes()) {
        if (node->get_landmark().is_true_in_state(init_state)) {
            /* All edges starting in initially true facts are not
               interesting for us since the cycles they possibly induce
               are already resolved initially. */
            continue;
        }
        size_t from_id = fact_to_action_lm_map[node->get_id()];
        assert(utils::in_bounds(from_id, lms));
        for (std::pair<LandmarkNode *const, EdgeType> &child : node->children) {
            size_t to_id = fact_to_action_lm_map[child.first->get_id()];
            assert(utils::in_bounds(to_id, lms));
            add_edge(from_id, to_id, child.second);
        }
    }
}

void DisjActionLandmarkGraph::add_required_edges(
    const LandmarkGraph &lm_graph, const LandmarkStatusManager &manager,
    const State &state, const std::vector<size_t> &fact_to_action_lm_map) {
    // TODO: check if all these conditions are correct.
    for (auto &node : lm_graph.get_nodes()) {
        int id = node->get_id();
        if (manager.get_landmark_status(id) == LandmarkStatus::ACCEPTED) {
            /*
              All edges starting in nodes that are accepted are not
              interesting for us since the cycles they possibly induce
              might already be resolved.
            */
            continue;
        }
        size_t from_id = fact_to_action_lm_map[id];
        assert(utils::in_bounds(from_id, lms));
        for (std::pair<LandmarkNode *const, EdgeType> &child : node->children) {
            int child_id = child.first->get_id();
            /*
              For any kind of first(A) < xy(B) ordering, if B is not
              required, then the ordering doesn't tell us anything.
            */
            if (manager.get_landmark_status(child_id) == ACCEPTED) {
                continue;
            }
            /*
              For a greedy-necessary ordering A -gn-> B (or stronger,
              i.e., necessary), it remains valid even if A was reached
              but does not hold in the current state.
              In all other cases, we skip this ordering.
            */
            if (manager.get_landmark_status(id) == ACCEPTED_AND_REQUIRED) {
                if (child.second < EdgeType::GREEDY_NECESSARY
                    || node->get_landmark().is_true_in_state(state)
                    || manager.get_landmark_status(child_id) != REQUIRED) {
                    continue;
                }
            }
            assert(manager.get_landmark_status(id) == REQUIRED
                   || (child.second >= EdgeType::GREEDY_NECESSARY
                       && !node->get_landmark().is_true_in_state(state)));
            /* TODO: for necessary orderings, there could be an even
                stronger criterion, but they are never generated in the
                code so we don't implement it here. */

            size_t to_id = fact_to_action_lm_map[child_id];
            assert(utils::in_bounds(to_id, lms));
            add_edge(from_id, to_id, child.second);
        }
    }
}

size_t DisjActionLandmarkGraph::add_node(const set<int> &actions) {
    auto it = ids.find(actions);
    if (it == ids.end()) {
        size_t id = ids.size();
        ids[actions] = id;
        lms.emplace_back(actions);
        return id;
    }
    return it->second;
}

void DisjActionLandmarkGraph::add_edge(size_t from_id, size_t to_id,
                                       EdgeType edge_type) {
    DisjActionLandmarkNode &dalm_node = lms[to_id];
    /* If there is an action which occurs in both landmarks, applying it
       resolves both landmarks as well as the ordering in one step.
       This special case (which is a consequence of the definition of
       reasonable orderings) makes a lot of things very complicated.
       Ignoring these cases may be desired sometimes which is why we do
       not take them over into our DALM-graph here. */
    if (edge_type >= EdgeType::NATURAL) {
        dalm_node.add_strong_dependency(from_id, num_orderings,
                                        num_weak_orderings);
    } else if (edge_type == EdgeType::REASONABLE
               && !lms[from_id].overlaps_with(dalm_node)) {
        dalm_node.add_weak_dependency(from_id, num_orderings,
                                      num_weak_orderings);
    }
}

void DisjActionLandmarkGraph::dump_lm(int id) const {
    cout << "lm" << id << ": <";
    for (int action : lms[id].actions) {
        cout << action << " ";
    }
    cout << ">" << endl;

    if (!lms[id].get_dependencies().empty()) {
        cout << "\tdepends on ";
        for (auto dep : lms[id].get_dependencies()) {
            cout << "lm" << dep.first << "(";
            switch (dep.second) {
            case OrderingType::strong:
                cout << "s";
                break;
            case OrderingType::weak:
                cout << "w";
                break;
            default:
                cout << "?";
                break;
            }
            cout << ") ";
        }
        cout << endl;
    }
}

void DisjActionLandmarkGraph::dump() const {
    cout << "== Disjunctive Action Landmark Graph ==" << endl;
    for (size_t id = 0; id < lms.size(); ++id) {
        dump_lm(id);
    }
    cout << "== End of Graph ==" << endl;
}

void DisjActionLandmarkGraph::dump_dot() const {
    cout << "digraph graphname {\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        cout << "  lm" << id << " [label=\"<";
        for (int a : lms[id].actions) {
            cout << a << " ";
        }
        cout << ">\"];\n";
    }
    cout << "\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        for (pair<int, OrderingType> dep : lms[id].get_dependencies()) {
            cout << "  lm" << dep.first << " -> lm" << id;
            if (dep.second == OrderingType::weak) {
                cout << " [style=dotted]";
            }
            cout << ";\n";
        }
    }
    cout << "}" << endl;
}

size_t DisjActionLandmarkGraph::get_number_of_landmarks() const {
    return lms.size();
}

OrderingType DisjActionLandmarkGraph::get_ordering_type(int from, int to) {
    assert(0 <= from && from < static_cast<int>(lms.size()));
    assert(0 <= to && to < static_cast<int>(lms.size()));
    assert(lms[to].depends_on(from));
    return lms[to].get_dependency(from);
}

const std::set<int> &DisjActionLandmarkGraph::get_actions(int id) {
    assert(0 <= id && id < static_cast<int>(lms.size()));
    return lms[id].actions;
}

const std::map<int, OrderingType> &
DisjActionLandmarkGraph::get_dependencies(int id) {
    assert(0 <= id && id < static_cast<int>(lms.size()));
    return lms[id].get_dependencies();
}

std::vector<std::map<int, bool>> DisjActionLandmarkGraph::to_adj_list() const {
    size_t n = get_number_of_landmarks();
    vector<map<int, bool>> adj(n, map<int, bool>{});
    for (size_t id = 0; id < n; ++id) {
        for (pair<int, OrderingType> dep : lms[id].get_dependencies()) {
            adj[dep.first][id] = dep.second == OrderingType::weak;
        }
    }
    return adj;
}
}
