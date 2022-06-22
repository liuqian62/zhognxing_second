#ifndef __UTILS_H__
#define __UTILS_H__
#include <string>
#include <vector>
// #include <algorithm>
using namespace std;

struct Position
{
    bool is_base_station=false;
    bool is_high_platform=false;
    bool is_drone=false;
    float x, y, z;
    int m = 0;
    int n = 0;
    int id=-1;
    void init(bool _is_base_station, float _x, float _y, float _z)
    {
        is_base_station = _is_base_station;
        x = _x;
        y = _y;
        z = _z;
    }
    bool operator==(Position pos)
    {
        if (is_base_station && pos.is_base_station)
        {
            if (id == pos.id)
            {
                return true;
            }
            else{
                return false;
            }
        }
        if(is_high_platform&&pos.is_high_platform){
            if(id==pos.id){
                return true;
            }
            else{
                return false;
            }
        }
        if(is_drone && pos.is_drone){
            if(m==pos.m&&n==pos.n){
                return true;
            }
            else{
                return false;
            }
        }
        return false;
    }

}
;

struct Period
{
    float start, end;
};

struct  Used_path
{
    Position pos1, pos2;
};


struct Disable_link
{
    Position pos1, pos2;
    vector<Period> periods;
};

struct Base_Station
{
    int id;
    float x, y, z;

    void init(int _id, float _x, float _y, float _z)
    {
        id = _id;
        x = _x;
        y = _y;
        z = _z;
    }
};

struct High_Platform
{
    int id;
    float x, y, z;
    void init(vector<int> plat, int _id)
    {
        id = _id;
        x = plat[0];
        y = plat[1];
        z = plat[2];
    }
    Position get_position(){
        Position pos;
        pos.id=id;
        pos.x=x;
        pos.y=y;
        pos.z=z;
        pos.is_high_platform=true;
        return pos;
    }
};

struct Demand
{
    Base_Station sta_station;
    Base_Station end_station;
    float time;
    int num_signals;
    void init(float _time, Base_Station _strat, Base_Station _end, int _num)
    {
        sta_station = _strat;
        end_station = _end;
        time = _time;
        num_signals = _num;
    }
};

struct Step
{
    string type;
    float t;
    int l=0;
    int m=0, n=0;
};

struct Solution
{
    float time;
    int start_id;
    int end_id;
    int num_signals;
    float delay;
    vector<Step> steps;
};

#endif