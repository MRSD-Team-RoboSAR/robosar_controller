#include <gtest/gtest.h>
#include <climits>
#include "lazy_traffic_rvo.hpp"

TEST(SingleAgentRVO, SingleAgentRVO)
{
    // ASSERT_EQ(3, add(1,2));

    // Local variables
    rvo_agent_obstacle_info_s agent_info = {"test_agent", RVO::Vector2(0.0, 0.0),
                                            RVO::Vector2(1.0, 0.0), RVO::Vector2(0.0, 0.0), 1.0};

    std::vector<rvo_agent_obstacle_info_s> neighbours_list;

    // Invoke RVO : should return preferred velocity as it is
    RVO::Vector2 new_velo = rvoComputeNewVelocity(agent_info, neighbours_list);
    ASSERT_FLOAT_EQ(1.0, new_velo.x());
    ASSERT_FLOAT_EQ(0.0, new_velo.y());
}

TEST(SingleAgentRVO, DoubleAgentRVONoCollision)
{
    // ASSERT_EQ(3, add(1,2));

    // Local variables
    rvo_agent_obstacle_info_s agent_info = {"test_agent", RVO::Vector2(0.0, 0.0),
                                            RVO::Vector2(1.0, 0.0), RVO::Vector2(0.0, 0.0), 1.0};

    // Add neighbour
    std::vector<rvo_agent_obstacle_info_s> neighbours_list;
    rvo_agent_obstacle_info_s neighbour_info = {"test_neighbour", RVO::Vector2(0.0, 0.0),
                                                RVO::Vector2(-1.0, 0.0), RVO::Vector2(100.0, 100.0), 1.0};
    neighbours_list.push_back(neighbour_info);

    // Invoke RVO : should return preferred velocity as it is
    RVO::Vector2 new_velo = rvoComputeNewVelocity(agent_info, neighbours_list);
    ASSERT_FLOAT_EQ(1.0, new_velo.x());
    ASSERT_FLOAT_EQ(0.0, new_velo.y());
}

TEST(DoubleAgentRVOCollisionAgentMoving, DoubleAgentRVOCollisionAgentMoving)
{
    // ASSERT_EQ(3, add(1,2));

    // Local variables
    rvo_agent_obstacle_info_s agent_info = {"test_agent", RVO::Vector2(1.0, 0.0),
                                            RVO::Vector2(0.0, 0.0), RVO::Vector2(0.0, 0.0), 1.0};

    // Add neighbour
    std::vector<rvo_agent_obstacle_info_s> neighbours_list;
    // rvo_agent_obstacle_info_s neighbour_info = {"test_neighbour",RVO::Vector2(0.0,0.0),
    //                             RVO::Vector2(-1.0,0.0),RVO::Vector2(5.0,0.0),1.0};
    // neighbours_list.push_back(neighbour_info);

    // // Invoke RVO
    // RVO::Vector2 new_velo = rvoComputeNewVelocity(agent_info, neighbours_list);
    // std::cout<<"New velocity: "<<new_velo.x()<<","<<new_velo.y()<<std::endl;
    // GTEST_ASSERT_NE(1.0, new_velo.x());
    // GTEST_ASSERT_NE(0.0, new_velo.y());

    // // Add neighbour closer
    // neighbours_list.clear();
    // rvo_agent_obstacle_info_s neighbour2_info = {"test_neighbour",RVO::Vector2(0.0,0.0),
    //                             RVO::Vector2(-1.0,0.0),RVO::Vector2(0.4,0.0),1.0};
    // neighbours_list.push_back(neighbour2_info);

    // // Invoke RVO
    // RVO::Vector2 new_velo2 = rvoComputeNewVelocity(agent_info, neighbours_list);
    // std::cout<<"New velocity: "<<new_velo2.x()<<","<<new_velo2.y()<<std::endl;
    // GTEST_ASSERT_NE(1.0, new_velo2.x());
    // GTEST_ASSERT_NE(0.0, new_velo2.y());
    // GTEST_ASSERT_LE(new_velo2.x(), new_velo.x());

    neighbours_list.clear();
    rvo_agent_obstacle_info_s neighbour3_info = {"test_neighbour_3", RVO::Vector2(1.0, 0.0),
                                                 RVO::Vector2(0.0, 0.0), RVO::Vector2(5.0, 0.0), 1.0};
    neighbours_list.push_back(neighbour3_info);

    // Invoke RVO
    RVO::Vector2 new_velo3 = rvoComputeNewVelocity(agent_info, neighbours_list);
    std::cout << "New velocity: " << new_velo3.x() << "," << new_velo3.y() << std::endl;
    GTEST_ASSERT_NE(1.0, new_velo3.x());
    // GTEST_ASSERT_NE(0.0, new_velo3.y());
    // GTEST_ASSERT_LE(new_velo3.x(), new_velo.x());
}

TEST(DoubleAgentRVOCollisionNeighborMoving, DoubleAgentRVOCollisionNeighborMoving)
{
    // ASSERT_EQ(3, add(1,2));

    // Local variables
    rvo_agent_obstacle_info_s agent_info = {"test_agent", RVO::Vector2(0.0, 0.0),
                                            RVO::Vector2(0.0, 0.0), RVO::Vector2(0.0, 0.0), 1.0};

    // Add neighbour
    std::vector<rvo_agent_obstacle_info_s> neighbours_list;
    rvo_agent_obstacle_info_s neighbour_info = {"test_neighbour", RVO::Vector2(-1.0, 0.0),
                                                RVO::Vector2(-1.0, 0.0), RVO::Vector2(5.0, 0.0), 1.0};
    neighbours_list.push_back(neighbour_info);

    // Invoke RVO
    RVO::Vector2 new_velo = rvoComputeNewVelocity(agent_info, neighbours_list);
    std::cout << "New velocity: " << new_velo.x() << "," << new_velo.y() << std::endl;
    GTEST_ASSERT_NE(0.0, new_velo.x());
    GTEST_ASSERT_NE(0.0, new_velo.y());

    // // Add neighbour closer
    neighbours_list.clear();
    rvo_agent_obstacle_info_s neighbour2_info = {"test_neighbour", RVO::Vector2(-1.0, 0.0),
                                                 RVO::Vector2(-1.0, 0.0), RVO::Vector2(0.4, 0.0), 1.0};
    neighbours_list.push_back(neighbour2_info);

    // Invoke RVO
    RVO::Vector2 new_velo2 = rvoComputeNewVelocity(agent_info, neighbours_list);
    std::cout << "New velocity: " << new_velo2.x() << "," << new_velo2.y() << std::endl;
    GTEST_ASSERT_NE(0.0, new_velo.x());
    GTEST_ASSERT_NE(0.0, new_velo.y());
    GTEST_ASSERT_LE(new_velo2.x(), new_velo.x());
    // GTEST_ASSERT_LE(new_velo2.y(), new_velo.y());
}

// TEST(NumberCmpTest, ShouldFail){
//     ASSERT_NE(INT_MAX, add(INT_MAX, 1));
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}