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

TEST(ObliqueCollision, ObliqueCollision){
    
    //ASSERT_EQ(3, add(1,2));

    // Define local variables
    RVO::Vector2 ray_start(0.0, 0.0);
    RVO::Vector2 ray_velo(1.0,0.2);
    RVO::Vector2 disc_centre(5.0, 0.0);
    float disc_radius = 1.0;
    bool collision = false;


    // Compute time to collision
    float time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    ASSERT_NE(RVO_INFTY, time);
    std::cout<<"Time to collision is "<<time<<std::endl;

    // Compute time to collision
    disc_radius = 3.0;
    ray_start = RVO::Vector2(0.0, 2.0);
    ray_velo = RVO::Vector2(1.0,0.0);
    time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    ASSERT_NE(RVO_INFTY, time);
    std::cout<<"Time to collision is "<<time<<std::endl;
}

// current position: 45.039467 : 9.643812, vel_a_t_b: -0.298092 : -0.262426, neigh pos: 44.489468 : 9.193812, radius: 0.300000
TEST(BFSCollision, BFSCollision){
    
    //ASSERT_EQ(3, add(1,2));
//  current position: 45.059994 : 9.999364, vel_a_t_b: 0.150701 : -0.136289, neigh pos: 44.709995 : 9.649364, radius: 0.300000
    // Define local variables
    // RVO::Vector2 ray_start(45.059994, 9.999364);
    // RVO::Vector2 ray_velo(-0.150701, 0.0);
    // RVO::Vector2 disc_centre(44.709995, 9.649364);
    RVO::Vector2 ray_start(45.05999, 9.999364);
    RVO::Vector2 ray_velo(0.150701, 0.0);
    RVO::Vector2 disc_centre(44.709995, 9.649364);
    float disc_radius = 0.400;
    bool collision = false;


    // Compute time to collision
    float time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    //ASSERT_NE(RVO_INFTY, time);
    std::cout<<"Time to collision is "<<time<<std::endl;

    // Compute time to collision
    // disc_radius = 3.0;
    // ray_start = RVO::Vector2(0.0, 2.0);
    // ray_velo = RVO::Vector2(1.0,0.0);
    // time = rvoTimeToCollision(ray_start, ray_velo, disc_centre, disc_radius, collision);
    // ASSERT_NE(RVO_INFTY, time);
    // std::cout<<"Time to collision is "<<time<<std::endl;
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}