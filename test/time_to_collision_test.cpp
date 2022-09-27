#include <gtest/gtest.h>
#include <climits>
#include "lazy_traffic_rvo.hpp"


TEST(SimpleCollision, SimpleCollision){
    
    //ASSERT_EQ(3, add(1,2));

    // Define local variables
    RVO::Vector2 ray_start(0.0, 0.0);
    RVO::Vector2 ray_velo(1.0,0.0);
    RVO::Vector2 disc_centre(5.0, 0.0);
    float disc_radius = 1.0;
    bool collision = false;


    // Compute time to collision
    float time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    ASSERT_FLOAT_EQ(4.0, time);


    // Compute time to collision
    disc_radius = 3.0;
    time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    ASSERT_FLOAT_EQ(2.0, time);
}

TEST(SimpleNoCollision, SimpleNoCollision){
     // Define local variables
    RVO::Vector2 ray_start(0.0, 0.0);
    RVO::Vector2 ray_velo(-1.0,0.0);
    RVO::Vector2 disc_centre(5.0, 0.0);
    float disc_radius = 1.0;
    bool collision = false;


    // Compute time to collision
    float time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    ASSERT_FLOAT_EQ(RVO_INFTY, time);


    // Compute time to collision
    RVO::Vector2 ray_velo2(1.0,0.0);
    RVO::Vector2 ray_start2(0.0, 1.0);
    time = rvoTimeToCollision(ray_start2, ray_velo2, disc_centre, disc_radius, collision);
    ASSERT_FLOAT_EQ(RVO_INFTY, time);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}