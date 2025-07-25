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

        const size_t N = nodes.size();
        std::vector<bool> blocked(N, false);
        std::unordered_map<size_t, std::vector<size_t>> block_map;
        std::vector<size_t> stack; // vertex stack for reconstructing cycles

        // Tarjan’s SCC algorithm
        auto tarjan_scc = [&](const std::vector<std::vector<Edge>> &graph) -> std::vector<std::vector<size_t>> {
            constexpr size_t UNVISITED = std::numeric_limits<size_t>::max();
            std::vector<size_t> index(graph.size(), UNVISITED), lowlink(graph.size());
            std::vector<bool> on_stack(graph.size(), false);
            std::stack<size_t> tarjan_stack;
            std::vector<std::vector<size_t>> sccs;
            size_t idx = 0;

            std::function<void(size_t)> strong_connect = [&](size_t v) {
                index[v] = lowlink[v] = idx++;
                tarjan_stack.push(v);
                on_stack[v] = true;

                for (const auto &edge : graph[v]) {
                    size_t w = edge.to;
                    if (index[w] == UNVISITED) {
                        strong_connect(w);
                        lowlink[v] = std::min(lowlink[v], lowlink[w]);
                    } else if (on_stack[w]) {
                        lowlink[v] = std::min(lowlink[v], index[w]);
                    }
                }

                if (lowlink[v] == index[v]) {
                    std::vector<size_t> scc;
                    size_t w;
                    do {
                        w = tarjan_stack.top();
                        tarjan_stack.pop();
                        on_stack[w] = false;
                        scc.push_back(w);
                    } while (w != v);
                    sccs.push_back(scc);
                }
            };

            for (size_t v = 0; v < graph.size(); ++v) {
                if (index[v] == UNVISITED) {
                    strong_connect(v);
                }
            }

            return sccs;
        };

        // Unblock a node recursively
        std::function<void(size_t)> unblock = [&](size_t u) {
            blocked[u] = false;
            for (auto &w : block_map[u]) {
                if (blocked[w]) {
                    unblock(w);
                }
            }
            block_map[u].clear();
        };

        // The main DFS-like recursive circuit finder
        std::function<bool(size_t, size_t)> circuit = [&](size_t v, size_t s) -> bool {
            bool found_cycle = false;
            stack.push_back(v);
            blocked[v] = true;

            for (const auto &edge : G[v]) {
                size_t w = edge.to;
                if (w == s) {
                    // Reconstruct cycle using current stack
                    std::vector<Edge> cycle;
                    for (size_t i = 0; i < stack.size(); ++i) {
                        size_t from = stack[i];
                        size_t to = stack[(i + 1) % stack.size()];
                        for (const auto &e : G[from]) {
                            if (e.to == to) {
                                cycle.push_back(e);
                                break;
                            }
                        }
                    }
                    result.push_back(std::move(cycle));
                    found_cycle = true;
                } else if (!blocked[w]) {
                    if (circuit(w, s)) {
                        found_cycle = true;
                    }
                }
            }

            if (found_cycle) {
                unblock(v);
            } else {
                for (const auto &edge : G[v]) {
                    size_t w = edge.to;
                    if (std::find(block_map[w].begin(), block_map[w].end(), v) == block_map[w].end()) {
                        block_map[w].push_back(v);
                    }
                }
            }

            stack.pop_back();
            return found_cycle;
        };

        // Johnson’s main loop
        std::vector<bool> removed(N, false);
        for (size_t start = 0; start < N; ++start) {
            // Build subgraph of unremoved nodes
            std::vector<std::vector<Edge>> subgraph(N);
            for (size_t u = 0; u < N; ++u) {
                if (removed[u])
                    continue;
                for (const auto &edge : G[u]) {
                    if (!removed[edge.to]) {
                        subgraph[u].push_back(edge);
                    }
                }
            }

            // Get SCCs of subgraph
            auto sccs = tarjan_scc(subgraph);

            // Find SCC with least vertex ≥ start that has size ≥ 2
            std::optional<std::vector<size_t>> selected_scc;
            size_t least_node = N;
            for (const auto &scc : sccs) {
                if (scc.size() < 2)
                    continue;
                size_t min_node = *std::min_element(scc.begin(), scc.end());
                if (min_node >= start && min_node < least_node) {
                    least_node = min_node;
                    selected_scc = scc;
                }
            }

            if (!selected_scc)
                break;

            size_t s = least_node;
            for (size_t v : *selected_scc) {
                blocked[v] = false;
                block_map[v].clear();
            }

            circuit(s, s);
            removed[s] = true; // Mark the node as removed
        }

        return result;
    }
};

} // namespace GraphicsLab