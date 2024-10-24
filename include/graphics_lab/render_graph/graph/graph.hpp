#pragma once

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
    }

    void add_directed_edge(size_t from, size_t to, EdgeAttachmentType data) {
        G[from].emplace_back(from, to, data);
    }

    void add_bidirectional_edge(size_t from, size_t to, EdgeAttachmentType data) {
        add_directed_edge(from, to, data);
        add_directed_edge(to, from, data);
    }
};

}