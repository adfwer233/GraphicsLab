#include "geometry/spatial_datastructure/hash_grid.hpp"

#include "gtest/gtest.h"

#include "geometry/spatial_datastructure/kd_tree.hpp"
#include "spdlog/spdlog.h"

#include "utils/sampler.hpp"

TEST(SpatialDataStructureTest, PrimitiveTest) {
    using namespace GraphicsLab::Geometry::KDTree;
    PointPrimitive<3> p1, p2;
    std::swap(p1, p2);
    std::vector<PointPrimitive<3>> v {p1, p2};
    std::ranges::sort(v, [](const auto l, const auto r) {return l.position[0] < r.position.x;});
}

TEST(SpatialDataStructureTest, KDTreeTest) {
    using namespace GraphicsLab::Geometry::KDTree;
    std::vector<PointPrimitive<3>> points = { {{1, 2, 3}}, {{4, 5, 6}}, {{7, 8, 9}}, {{2, 3, 4.5}}, {{5, 6, 7}} };
    KDTree<3, PointPrimitive<3>> tree(points);

    glm::vec3 queryPoint = { 2, 3, 4 };
    auto nearest = tree.nearestNeighbor(tree.root, queryPoint);

    spdlog::info("{} {} {}", nearest.position.x, nearest.position.y, nearest.position.z);
}

TEST(SpatialDataStructureTest, KDTreePointFuzzyTest) {
    using namespace GraphicsLab::Geometry::KDTree;
    using namespace GraphicsLab;

    std::vector<PointPrimitive<3>> points;

    int sample_num = 10000;

    for (int i = 0; i < sample_num; i++) {
        points.emplace_back(glm::vec3{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()});
    }

    glm::vec3 test_point{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};

    KDTree<3, PointPrimitive<3>> tree(points);
    auto nearest_by_kd_tree = tree.nearestNeighbor(tree.root, test_point);

    float distance_brute_force = std::numeric_limits<float>::max();
    for (auto& p: points) {
        distance_brute_force = std::min(distance_brute_force, p.distance_to(test_point));
    }

    float distance_kd_tree = nearest_by_kd_tree.distance_to(test_point);

    EXPECT_FLOAT_EQ(distance_kd_tree, distance_brute_force);
}

TEST(SpatialDataStructureTest, KDTreeBallFuzzyTest) {
    using namespace GraphicsLab::Geometry::KDTree;
    using namespace GraphicsLab;

    std::vector<BallPrimitive<3>> balls;

    int sample_num = 512 * 512;
    for (int i = 0; i < sample_num; i++) {
        balls.emplace_back(glm::vec3{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()}, Sampler::sampleUniform(0.1, 0.2));
    }

    glm::vec3 test_point{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};

    KDTree<3, BallPrimitive<3>> tree(balls);
    std::vector<BallPrimitive<3>> covering_balls;

    spdlog::critical("start 1");
    tree.query(tree.root, test_point, -1e-3, covering_balls, 0, 128);

    spdlog::critical("start 2");

    std::vector<BallPrimitive<3>> covering_balls_brute_force;

    for (auto& b: balls) {
        if (b.distance_to(test_point) < -1e-3) {
            covering_balls_brute_force.push_back(b);
        }
    }

    spdlog::critical("# covering balls: {}", covering_balls.size());
    EXPECT_EQ(covering_balls.size(), covering_balls_brute_force.size());
}

TEST(SpatialDataStructureTest, HashGridTest) {
    using namespace GraphicsLab::Geometry::KDTree;
    using namespace GraphicsLab;

    std::vector<BallPrimitive<3>> balls;

    int sample_num = 512 * 512;
    for (int i = 0; i < sample_num; i++) {
        balls.emplace_back(glm::vec3{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()}, Sampler::sampleUniform(0.1, 0.2));
    }

    glm::vec3 test_point{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};

    GraphicsLab::Geometry::HashGrid::HashGrid hash_grid(0.1);

    spdlog::info("start 0");
    for (int i = 0; i < sample_num; i++) {
        hash_grid.addBall({balls[i].center, balls[i].radius});
    }

    spdlog::info("start 1");
    auto covering_balls = hash_grid.query(test_point);

    spdlog::info("start 2");

    std::vector<BallPrimitive<3>> covering_balls_brute_force;

    for (auto& b: balls) {
        if (b.distance_to(test_point) <= 0) {
            covering_balls_brute_force.push_back(b);
        }
    }

    spdlog::critical("# covering balls: {}", covering_balls.size());
    EXPECT_EQ(covering_balls.size(), covering_balls_brute_force.size());
}

TEST(SpatialDataStructureTest, KDTreeLineSegmentFuzzyTest) {
    GTEST_SKIP();
    using namespace GraphicsLab::Geometry::KDTree;
    using namespace GraphicsLab;

    std::vector<LineSegmentPrimitive<3>> segments;

    constexpr int sample_num = 10000;

    for (int i = 0; i < sample_num; i++) {
        glm::vec3 p1{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};
        glm::vec3 p2{Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};
        segments.emplace_back(p1, p2);
    }

    glm::vec3 test_point = {Sampler::sampleUniform(), Sampler::sampleUniform(), Sampler::sampleUniform()};


    KDTree<3, LineSegmentPrimitive<3>> tree(segments);

    auto nearest_by_kd_tree = tree.nearestNeighbor(tree.root, test_point);

    float distance_brute_force = std::numeric_limits<float>::max();
    for (auto& p: segments) {
        distance_brute_force = std::min(distance_brute_force, p.distance_to(test_point));
    }

    const float distance_kd_tree = nearest_by_kd_tree.distance_to(test_point);

    EXPECT_FLOAT_EQ(distance_kd_tree, distance_brute_force);
}