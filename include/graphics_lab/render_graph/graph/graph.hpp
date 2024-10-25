#pragma once

#include <algorithm>
#include <functional>
#include <ranges>
#include <stack>
#include <stdexcept>
#include <vector>

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
};

} // namespace GraphicsLab