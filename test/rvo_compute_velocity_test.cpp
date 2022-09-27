#include <gtest/gtest.h>
#include <climits>
#include "lazy_traffic_rvo.hpp"

TEST(SingleAgentRVO, SingleAgentRVO){
    //ASSERT_EQ(3, add(1,2));

    // Local variables
    rvo_agent_info_s agent_info = {"test_agent",RVO::Vector2(0.0,0.0),
                                RVO::Vector2(1.0,0.0),RVO::Vector2(0.0,0.0),1.0};
    
    std::vector<rvo_agent_info_s> neighbours_list;

    // Invoke RVO : should return preferred velocity as it is
    RVO::Vector2 new_velo = rvoComputeNewVelocity(agent_info, neighbours_list);
    ASSERT_FLOAT_EQ(1.0, new_velo.x());
    ASSERT_FLOAT_EQ(0.0, new_velo.y());

}

// TEST(NumberCmpTest, ShouldFail){
//     ASSERT_NE(INT_MAX, add(INT_MAX, 1));
// }

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}