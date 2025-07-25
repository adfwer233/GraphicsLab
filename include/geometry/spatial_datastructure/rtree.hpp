#pragma once

#include "glm/glm.hpp"
#include <stack>

namespace GraphicsLab::Geometry {

template <size_t dim, typename AttachmentType> class RTree {
    using PointType = std::conditional_t<dim == 2, glm::dvec2, glm::dvec3>;

    /**
     * Sphere region.
     */
    struct Sphere {
        PointType center;
        double radius;

        Sphere() : radius(0) {
        }
        Sphere(const PointType &c, double r) : center(c), radius(r) {
        }

        bool contains(const PointType &p) const {
            return distance(center, p) < radius;
        }

        static double distance(const PointType &a, const PointType &b) {
            return glm::distance(a, b);
        }

        void combine(const Sphere *a) {
            for (int i = 0; i < dim; i++) {
                center[i] = (a->center[i] + center[i]) / 2.0;
            }
            radius = distance(center, a->center) + std::max(a->radius, radius);
        }

        static Sphere combine(const Sphere *a, const Sphere *b) {
            PointType newCenter = (a->center + b->center) / 2.0;
            double newRadius = distance(newCenter, a->center) + std::max(a->radius, b->radius);
            return Sphere(newCenter, newRadius);
        }
    };

    struct Node {
        std::vector<Node *> children_;
        Node *parent_;
        AttachmentType point_;
        Sphere *mbr_;

        double tol_;

        Node(Sphere *s = new Sphere(), AttachmentType p = AttachmentType{}, Node *parent = nullptr, double tol = 1e-6)
            : mbr_(s), point_(p), parent_(parent), tol_(tol) {
        }

        void updateSphere() {
            if (children_.size() > 0) {
                mbr_->center = children_[0]->mbr_->center;
                mbr_->radius = children_[0]->mbr_->radius;

                if (children_.size() > 1) {
                    for (int i = 1; i < children_.size(); i++) {
                        mbr_->combine(children_[i]->mbr_);
                    }
                }
            }
        }

        ~Node() {
            delete mbr_;
            for (auto &child : children_) {
                delete child;
            }
        }
    };

    Node *root_;

    const int maxEntries_ = 4;

    double tol_;

  public:
    RTree(double tolerance = 1e-6) : root_(new Node()), tol_(tolerance) {
    }

    ~RTree() {
        delete root_;
    }

    void insert(const PointType &position, AttachmentType data, double pos_tol = 1e-6) {
        pos_tol = std::max(pos_tol, tol_);

        Node *newNode = new Node(new Sphere(position, pos_tol), data, nullptr, pos_tol);

        std::stack<Node *> stack;
        stack.push(root_);
        while (stack.top()->children_.size() > 0) {
            Node *best_child = chooseSubtree(stack.top(), newNode);
            stack.push(best_child);
        }

        Node *leaf = stack.top();
        newNode->parent_ = leaf->parent_;
        if (leaf != root_) {
            leaf->parent_->children_.push_back(newNode);
        } else {
            root_->children_.push_back(newNode);
            newNode->parent_ = root_;
        }

        while (!stack.empty()) {
            Node *top = stack.top();
            top->tol_ = std::max(top->tol_, newNode->tol_);
            if (top->children_.size() > maxEntries_) {
                Node *new_split_node = nullptr;
                split(top, new_split_node);
                new_split_node->parent_ = top->parent_;

                top->updateSphere();
                new_split_node->updateSphere();

                if (top != root_) {
                    top->parent_->children_.push_back(new_split_node);
                } else {
                    // root
                    Node *new_root = new Node();

                    new_root->children_.push_back(root_);
                    new_root->children_.push_back(new_split_node);

                    new_root->tol_ = std::max(root_->tol_, new_split_node->tol_);

                    root_->parent_ = new_root;
                    new_split_node->parent_ = new_root;

                    root_ = new_root;
                    new_root->updateSphere();
                }
            } else if (top->children_.size() > 0) {
                top->updateSphere();
            }
            stack.pop();
        }
    }

  private:
    Node *chooseSubtree(Node *node, Node *newNode) {
        Node *bestChild = nullptr;
        double bestIncrease = std::numeric_limits<double>::max();
        for (auto &child : node->children_) {
            Sphere combinedSphere = Sphere::combine(child->mbr_, newNode->mbr_);
            double increase = combinedSphere.radius - child->mbr_->radius;
            if (increase < bestIncrease) {
                bestIncrease = increase;
                bestChild = child;
            }
        }
        return bestChild;
    }

    void split(Node *node, Node *&newNode) {
        // Implementing a more sophisticated split algorithm (e.g., linear split)
        newNode = new Node();
        // Find split index
        size_t splitIndex = node->children_.size() / 2;
        newNode->children_.insert(newNode->children_.end(),
                                  std::make_move_iterator(node->children_.begin() + splitIndex),
                                  std::make_move_iterator(node->children_.end()));
        for (auto *&n : newNode->children_) {
            n->parent_ = newNode;
            newNode->tol_ = std::max(newNode->tol_, n->tol_);
        }
        node->children_.erase(node->children_.begin() + splitIndex, node->children_.end());
    }

    Node *findPointNode(const PointType &point, double point_tol = 1e-6) {
        point_tol = std::max(point_tol, tol_);
        if (root_->children_.size() > 0) {
            std::stack<Node *> stack;
            stack.push(root_);
            while (!stack.empty()) {
                Node *current = stack.top();
                stack.pop();
                if (current->mbr_->contains(point)) {
                    if (current->children_.empty() and
                        same_point(current->mbr_->center, point, (current->tol_ + point_tol) / 2)) {
                        return current;
                    }
                    for (auto &child : current->children_) {
                        stack.push(child);
                    }
                }
            }
        }
        return nullptr;
    }

  public:
    bool same_point(const PointType &p, const PointType &q, double tol = 1e-6) {
        return glm::distance(p, q) < tol;
    }

    bool findPointInRange(const PointType &point, AttachmentType &foundPoint, double point_tol = 1e-6) {
        Node *node = findPointNode(point, point_tol);
        if (node != nullptr) {
            foundPoint = node->point_;
            return true;
        }
        return false;
    }

    bool findPointInRange(const PointType &point, double point_tol = 1e-6) {
        AttachmentType foundPoint{};
        Node *node = findPointNode(point, point_tol);
        if (node != nullptr) {
            foundPoint = node->point_;
            return true;
        }
        return false;
    }

    void setTolerance(double tol) {
        tol_ = tol;
    }

    bool replacePoint(const PointType &point, AttachmentType data, double point_tol = 1e-6) {
        Node *node = findPointNode(point, point_tol);
        if (node != nullptr) {
            node->point_ = data;
            return true;
        }
        return false;
    }
};

} // namespace GraphicsLab::Geometry