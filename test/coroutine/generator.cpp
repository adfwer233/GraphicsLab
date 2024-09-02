//#include "coroutine/generator.hpp"
//
//#include "iostream"
//#include "experimental/generator"
//
//#include "gtest/gtest.h"
//
//template<typename T>
//struct Tree
//{
//    T value;
//    Tree *left{}, *right{};
//
//    [[nodiscard]] Generator<const T> traverse_inorder() const {
//        if (left)
//            for (const T& x : left->traverse_inorder())
//                co_yield x;
//
//        co_yield value;
//        if (right)
//            for (const T& x : right->traverse_inorder())
//                co_yield x;
//    }
//};
//
//Generator<size_t> sequence_generator() {
//    size_t threshold = 3;
//    size_t n = 20;
//    for (size_t i = 0; i < n; ) {
//        if (i > threshold && i < n - threshold) {
//            i += threshold;
//        } else {
//            i++;
//        }
//        co_yield i;
//    }
//}
//
//Generator<size_t> squared_sequence() {
//    for(auto i: sequence_generator())
//        co_yield i * i;
//}
//
//TEST(CoroutineTest, GeneratorTest) {
//    Tree<char> tree[]
//            {
//                    {'D', tree + 1, tree + 2},
//                    //                            │
//                    //            ┌───────────────┴────────────────┐
//                    //            │                                │
//                    {'B', tree + 3, tree + 4},       {'F', tree + 5, tree + 6},
//                    //            │                                │
//                    //  ┌─────────┴─────────────┐      ┌───────────┴─────────────┐
//                    //  │                       │      │                         │
//                    {'A'},                  {'C'}, {'E'},                    {'G'}
//            };
//
//    for (auto x: tree->traverse_inorder()) {
//        std::cout << x << ' ';
//    }
//    std::cout << std::endl;
//
//
//    for (auto i: sequence_generator()) {
//        std::cout << i << ' ';
//    }
//    std::cout << std::endl;
//
//    for (auto i : squared_sequence()) {
//        std::cout << i << ' ';
//    }
//    std::cout << std::endl;
//}