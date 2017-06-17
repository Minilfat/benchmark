
#ifndef BENCHMARK_SPATIAL_TREE_NODE_H
#define BENCHMARK_SPATIAL_TREE_NODE_H

#include <utility>
#include <vector>
#include "bound.h"

namespace bdm {

    using std::vector;
    using std::pair;

/// Is a core class, which provides with variety of virtual
/// functions for children-type trees and an implemented search neighbours
/// function,
/// which is common for all the trees.
    template <typename T>
    class SpatialTreeNode {
    public:


        explicit SpatialTreeNode(const Bound &bound) : bound_(bound) {}

        SpatialTreeNode() {};

        virtual ~SpatialTreeNode() {}

        ///  @tparam T - type of the object
        ///  @return bounds of the node
        const Bound &GetBound() const;

        virtual bool IsLeaf() const = 0;

        virtual void Put(Point const &p, T obj) = 0;

        ///  @tparam T - type of the object
        ///  @param distance - distance to search within
        ///  @return
        virtual vector<pair<T, T> > GetNeighbors(double distance) const;

    protected:
        Bound bound_;

    private:
        virtual SpatialTreeNode<T> **GetChildrenNodes() const = 0;

        virtual size_t GetChildrenSize() const = 0;

        virtual const vector<pair<Point, T> > &GetObjects() const = 0;

        ///  Neighbour search function
        ///  Search pairs (a, b) where a from A and b from B
        ///  Absolutelly the same method. The only difference is result type
        ///  @tparam T
        ///  @param a - node of the tree
        ///  @param b - node of the tree
        ///  @param distance - finding neighbors within that distance
        ///  @param result - container, where we save retrieved data (all neighbor
        ///  pairs
        ///  without its points)
        static void GetNeighbors(SpatialTreeNode<T> const *a,
                                 SpatialTreeNode<T> const *b, double distance,
                                 vector<pair<T, T> > *result);
    };

    template <typename T>
    const Bound &SpatialTreeNode<T>::GetBound() const {
        return bound_;
    }

    template <typename T>
    vector<pair<T, T> > SpatialTreeNode<T>::GetNeighbors(double distance) const {
        vector<pair<T, T> > result;
        GetNeighbors(this, this, distance * distance, &result);
        return result;
    }

    template <typename T>
    void SpatialTreeNode<T>::GetNeighbors(SpatialTreeNode<T> const *a,
                                          SpatialTreeNode<T> const *b,
                                          double distance,
                                          vector<pair<T, T> > *result) {
        if (a->IsLeaf() && b->IsLeaf()) {
            const auto &a_objs = a->GetObjects();
            const auto &b_objs = b->GetObjects();

            bool is_same = a == b;

            for (size_t i = 0; i < a_objs.size(); i++) {
                for (size_t j = is_same ? (i + 1) : 0; j < b_objs.size(); j++) {
                    if ((a_objs.at(i).first).SquaredEuclidianDistance(b_objs.at(j).first) <=
                        distance) {
                        result->push_back(
                                make_pair(a_objs.at(i).second, b_objs.at(j).second));
                    }
                }
            }
        } else {
            SpatialTreeNode<T> **a_nodes, **b_nodes;
            int a_size = 0, b_size = 0;
            if (a->IsLeaf()) {
                b_nodes = b->GetChildrenNodes();
                for (size_t i = 0; i < b->GetChildrenSize(); i++) {
                    if (a->GetBound().SquaredDistance(b_nodes[i]->GetBound()) <= distance) {
                        GetNeighbors(a, b_nodes[i], distance, result);
                    }
                }
            } else if (b->IsLeaf()) {
                a_nodes = a->GetChildrenNodes();
                for (size_t i = 0; i < a->GetChildrenSize(); i++) {
                    if (a_nodes[i]->GetBound().SquaredDistance(b->GetBound()) <= distance) {
                        GetNeighbors(a_nodes[i], b, distance, result);
                    }
                }
            } else {
                a_nodes = a->GetChildrenNodes();
                b_nodes = b->GetChildrenNodes();
                a_size = a->GetChildrenSize();
                b_size = b->GetChildrenSize();
                bool is_same = a == b;
                for (int i = 0; i < a_size; i++) {
                    for (int j = is_same ? (i) : 0; j < b_size; j++) {
                        if (a_nodes[i]->GetBound().SquaredDistance(b_nodes[j]->GetBound()) <=
                            distance) {
                            GetNeighbors(a_nodes[i], b_nodes[j], distance, result);
                        }
                    }
                }
            }
        }
    }


}


#endif //BENCHMARK_SPATIAL_TREE_NODE_H_H
