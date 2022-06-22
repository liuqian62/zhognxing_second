#ifndef __DRONE_H__
#define __DRONE_H__
#include <string>
#include "utils.hpp"
#include <math.h>
#include <iostream>
#include <queue>
// #include <vector>
#include <algorithm>
#include <unordered_map>
using namespace std;

class Drone
{
private:
    float _drone_drone = 115.0;
    float _drone_base = 70.0;
    float _v = 5.0;
    float _x_intraorbit = 90.0;
    float _y_interorbit = 80.0;
    float _H = 10.0;
    float _tf = 0.1;
    float _xishu = 0.0001;


    bool y_first=true;

    

public:

    vector<Disable_link> dis_links;
    vector<High_Platform> platforms;
    vector<High_Platform> useful_plat;
    vector<Position> sta_mns;
    vector<Position> des_mns;
    vector<Position> path_points;
    vector<bool> is_points_arive;
    vector<bool>is_plat_used;
    vector<bool>is_des_mn_used;
    /**
     * @brief Construct a new Drone object
     *
     */
    Drone();



    void get_platforms(vector<High_Platform>_platforms);

    /**
     * @brief at every position, find the best next drone or base station
     * 
     * @param stp one step, include t m n
     * @param time_now the time when this drone get the signal
     * @param position_now the position of signal
     * @param destination the destination base station position
     * @param finish finished the process or not
     */
    void find_best_drone(
        Step &stp,
        float &time_now,
        Position &position_now,
        Position destination,
        bool &finish);


    float get_dely(Position pos1,Position pos2,float start_time,float need_time);

    vector<Position> find_surrounding(Position pos,float time);

    void get_candidate_point(Position start,Position end,float star_time);

    void reset();

    void mysort(vector<Position> &surs,Position des);

    void search_by_bfs(
        vector<Step> &stps,
        float &time_now,
        Position position_now,
        Position destination
    );

    void find_step(
        Step &stp,
        float &time_now,
        Position &position_now,
        Position destination,
        bool &finish,
        int sta_mn_id,
        int plt_id1,
        int plt_id2,
        int des_mn_id);

    float is_link_avilable(Position pos1,Position pos2,float time_now);

    bool is_distance_suit(Position pos1,Position pos2,float &time_now);

    float get_distance(Position pos1,Position pos2);

    Position get_position_from_mn(int m,int n,float time_now);

    /**
     * @brief Destroy the Drone object
     *
     */
    ~Drone();
};

#endif