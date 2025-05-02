#pragma once

#include <algorithm>
#include <functional>
#include <ranges>
#include <stack>
#include <stdexcept>
#include <vector>

#include "spdlog/spdlog.h"

namespace GraphicsLab {

template <typename NodeAttachmentType, typename EdgeAttachmentType> struct DirectedGraph {
    struct Node {
        NodeAttachmentType data;
    };

    struct Edge {
        size_t from, to;
        EdgeAttachmentType data;
    };

    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> G;

    explicit DirectedGraph(size_t max_node) {
        G.resize(max_node);
    }

    size_t add_node(NodeAttachmentType node_data) {
        nodes.emplace_back(node_data);
        return nodes.size() - 1;
    }

    void add_directed_edge(size_t from, size_t to, EdgeAttachmentType data) {
        G[from].emplace_back(from, to, data);
    }

    void add_bidirectional_edge(size_t from, size_t to, EdgeAttachmentType data) {
        add_directed_edge(from, to, data);
        add_directed_edge(to, from, data);
    }

    void print_graph_info() const {

        if constexpr (IsStaticReflectedType<NodeAttachmentType>) {
            for (int i = 0; i < nodes.size(); i++) {
                spdlog::info("Node {}: {}", i, StaticReflect::serialization(nodes[i].data).dump());
            }
        }

        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < G[i].size(); j++) {
                auto e = G[i][j];
                spdlog::info("{} -> {}", e.from, e.to);
            }
        }
    }

    // Topological sort function
    std::vector<size_t> topological_sort() {
        size_t n = nodes.size();
        std::vector<int> visited(n, 0); // 0 = unvisited, 1 = visiting, 2 = visited
        std::stack<size_t> result;      // stores the topological ordering
        bool has_cycle = false;

        // Helper function to perform DFS
        std::function<void(size_t)> dfs = [&](size_t node) {
            if (visited[node] == 1) {
                has_cycle = true; // Found a back edge, so it's not a DAG
                return;
            }
            if (visited[node] == 2)
                return; // already visited

            visited[node] = 1; // mark as visiting

            for (const auto &edge : G[node]) {
                dfs(edge.to); // visit neighbors
            }

            visited[node] = 2; // mark as fully visited
            result.push(node); // push node to result in post-order
        };

        // Perform DFS from all unvisited nodes
        for (size_t i = 0; i < n; ++i) {
            if (visited[i] == 0) {
                dfs(i);
            }
        }

        // If a cycle is detected, throw an exception
        if (has_cycle) {
            throw std::runtime_error("Graph contains a cycle, topological sort is not possible.");
        }

        // Extract the topological order from the stack
        std::vector<size_t> topo_order;
        while (!result.empty()) {
            topo_order.push_back(result.top());
            result.pop();
        }

        return topo_order;
    }

    /**
     * @brief find all simple circuits in the graph
     *
     * c.f. Johnson Algorithm, FINDING ALL THE ELEMENTARY CIRCUITS OF A DIRECTED GRAPH
     *
     * @return vector of vector of edge, each inner vector represents a circuit
     */
    std::vector<std::vector<Edge>> find_all_simple_circuits() {
        std::vector<std::vector<Edge>> result;
        std::vector<bool> blocked(nodes.size(), false);
        std::unordered_map<size_t, std::vector<size_t>> block_map;
        std::stack<size_t> stack;
        std::vector<size_t> stack_vec;

        std::function<void(size_t)> unblock = [&](size_t node) {
            blocked[node] = false;
            while(!block_map[node].empty()) {
                size_t w = block_map[node].back();
                block_map[node].pop_back();
                if(blocked[w]) {
                    unblock(w);
                }
            }
        };

        std::function<bool(size_t, size_t)> circuit = [&](size_t v, size_t start) -> bool {
            bool found_cycle = false;
            stack.push(v);
            blocked[v] = true;
            stack_vec.push_back(v);

            for(const auto& edge: G[v]) {
                size_t w = edge.to;
                if(w == start) {
                    std::vector<Edge> cycle;

                    /**
                     * @todo: use map here
                     */
                    for(size_t i = 0; i < stack_vec.size(); ++i) {
                        size_t next = stack_vec[(i + 1) % stack_vec.size()];
                        for(auto ed: G[stack_vec[i]]) {
                            if(ed.to == next) {
                                cycle.push_back(ed);
                                break;
                            }
                        }
                    }
                    result.push_back(cycle);
                    found_cycle = true;
                } else if(!blocked[w]) {
                    if(circuit(w, start)) {
                        found_cycle = true;
                    }
                }
            }

            if(found_cycle) {
                unblock(v);
            } else {
                for(const auto& edge: G[v]) {
                    size_t w = edge.to;
                    if(std::find(block_map[w].begin(), block_map[w].end(), v) == block_map[w].end()) {
                        block_map[w].push_back(v);
                    }
                }
            }

            stack.pop();
            stack_vec.pop_back();
            return found_cycle;
        };

        auto tarjan_scc = [&](const std::vector<std::vector<Edge>>& graph) -> std::vector<std::vector<size_t>> {
            std::vector<std::vector<size_t>> sccs;
            std::vector<size_t> index(graph.size(), -1);
            std::vector<size_t> lowlink(graph.size(), -1);
            std::vector<bool> on_stack(graph.size(), false);
            std::stack<size_t> tarjan_stack;
            size_t current_index = 0;

            std::function<void(size_t)> strong_connect = [&](size_t v) -> void {
                index[v] = lowlink[v] = current_index++;
                tarjan_stack.push(v);
                on_stack[v] = true;

                for(const auto& edge: graph[v]) {
                    size_t w = edge.to;
                    if(index[w] == -1) {
                        strong_connect(w);
                        lowlink[v] = std::min(lowlink[v], lowlink[w]);
                    } else if(on_stack[w]) {
                        lowlink[v] = std::min(lowlink[v], index[w]);
                    }
                }

                if(lowlink[v] == index[v]) {
                    std::vector<size_t> scc;
                    size_t w;
                    do {
                        w = tarjan_stack.top();
                        tarjan_stack.pop();
                        on_stack[w] = false;
                        scc.push_back(w);
                    } while(v != w);
                    sccs.push_back(scc);
                }
            };

            for(size_t i = 0; i < graph.size(); ++i) {
                if(index[i] == -1) {
                    strong_connect(i);
                }
            }

            return sccs;
        };

        size_t start_index = 0;
        while(start_index < nodes.size()) {
            std::vector<std::vector<Edge>> subgraph(nodes.size());
            for(size_t i = start_index; i < nodes.size(); ++i) {
                for(const auto& edge: G[i]) {
                    if(edge.to >= start_index) {
                        subgraph[i].push_back(edge);
                    }
                }
            }

            auto sccs = tarjan_scc(subgraph);
            if(sccs.empty()) {
                break;
            }

            for(const auto& scc: sccs) {
                if(scc.size() > 1) {
                    start_index = *std::min_element(scc.begin(), scc.end());
                    break;
                }
            }

            if(sccs.front().size() == 1) {
                start_index++;
            } else {
                size_t s = sccs.front().front();
                for(const auto& node: sccs.front()) {
                    blocked[node] = false;
                    block_map[node].clear();
                }
                circuit(s, s);
                start_index++;
            }
        }

        return result;
    }
};

} // namespace GraphicsLab