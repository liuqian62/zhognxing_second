#include <iomanip>
#include <fstream>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
// #include <vector>
#include <algorithm>
#include <unordered_map>
using namespace std;

struct Position
{
    bool is_base_station = false;
    bool is_high_platform = false;
    bool is_drone = false;
    float x, y, z;
    int m = 0;
    int n = 0;
    int id = -1;
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
            else
            {
                return false;
            }
        }
        if (is_high_platform && pos.is_high_platform)
        {
            if (id == pos.id)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        if (is_drone && pos.is_drone)
        {
            if (m == pos.m && n == pos.n)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }
};

struct Period
{
    float start, end;
};

struct Used_path
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
    Position get_position()
    {
        Position pos;
        pos.id = id;
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.is_high_platform = true;
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
    int l = 0;
    int m = 0, n = 0;
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

    bool y_first = true;

public:
    vector<Disable_link> dis_links;
    vector<High_Platform> platforms;
    vector<High_Platform> useful_plat;
    vector<Position> sta_mns;
    vector<Position> des_mns;
    vector<Position> path_points;
    vector<bool> is_points_arive;
    vector<bool> is_plat_used;
    vector<bool> is_des_mn_used;
    /**
     * @brief Construct a new Drone object
     *
     */
    Drone();

    void get_platforms(vector<High_Platform> _platforms);

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

    float get_dely(Position pos1, Position pos2, float start_time, float need_time);

    vector<Position> find_surrounding(Position pos, float time);

    void get_candidate_point(Position start, Position end, float star_time);

    void reset();

    void mysort(vector<Position> &surs, Position des);

    void search_by_bfs(
        vector<Step> &stps,
        float &time_now,
        Position position_now,
        Position destination);

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

    float is_link_avilable(Position pos1, Position pos2, float time_now);

    bool is_distance_suit(Position pos1, Position pos2, float &time_now);

    float get_distance(Position pos1, Position pos2);

    Position get_position_from_mn(int m, int n, float time_now);

    /**
     * @brief Destroy the Drone object
     *
     */
    ~Drone();
};

Drone::Drone()
{
}
void Drone::mysort(vector<Position> &surs, Position des)
{
    int length = surs.size();
    vector<float> distan;
    for (int m = 0; m < length; m++)
    {
        distan.push_back(get_distance(surs[m], des));
    }

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (distan[j] > distan[j + 1])
            {
                int temp = distan[j];
                distan[j] = distan[j + 1];
                distan[j + 1] = temp;

                Position ind_temp = surs[j];
                surs[j] = surs[j + 1];
                surs[j + 1] = ind_temp;
            }
        }
    }
}

void Drone::search_by_bfs(
    vector<Step> &stps,
    float &time_now,
    Position position_now,
    Position destination)
{
    unordered_map<int, int> visited;
    vector<Used_path> used_paths;
    queue<vector<Step>> Q;
    vector<Step> best_path;
    float best_arive_time = 100.0f;
    vector<Position> sur = find_surrounding(position_now, time_now);
    mysort(sur, destination);
    // cout<<sur.size()<<"....."<<endl;
    for (uint i = 0; i < sur.size(); i++)
    {
        vector<Step> path;
        Step stp;
        float move = get_distance(position_now, sur[i]);
        if (sur[i].is_high_platform)
        {
            stp.type = "high";
            stp.l = sur[i].id;
            stp.t = time_now + _tf + _xishu * move + get_dely(position_now, sur[i], time_now, 0.115);
            visited.emplace(10000 * sur[i].id, 1);
        }
        else
        {
            stp.m = sur[i].m;
            stp.n = sur[i].n;
            stp.t = time_now + _tf + _xishu * move + get_dely(position_now, sur[i], time_now, 0.115);
            visited.emplace(100 * sur[i].m + sur[i].n, 1);
        }
        Used_path used_path;
        used_path.pos1 = position_now;
        used_path.pos2 = sur[i];
        used_paths.push_back(used_path);
        path.push_back(stp);
        Q.push(path);
    }
    while (!Q.empty())
    {
        // cout << Q.size() << endl;
        // if(Q.size()>10000){
        //     break;
        // }
        vector<Step> tmp_stps = Q.front();
        // cout<<tmp_stps.size()<<endl;
        // if(tmp_stps.size()>30){
        //     break;
        // }
        Q.pop();
        Step tmp_stp = tmp_stps[tmp_stps.size() - 1];
        Position tmp_pos;
        if (tmp_stp.type == "high")
        {
            tmp_pos = platforms[tmp_stp.l].get_position();
        }
        else
        {
            tmp_pos = get_position_from_mn(tmp_stp.m, tmp_stp.n, tmp_stp.t);
        }
        float tmp_t = tmp_stp.t;
        if (is_distance_suit(tmp_pos, destination, tmp_t))
        // if(get_distance(tmp_pos,destination)<70)
        {
            best_arive_time = tmp_t;
            best_path = tmp_stps;
            break;
            // cout<<"end"<<endl;
        }
        else if (tmp_t > best_arive_time)
        {
        }
        else
        {
            vector<Position> tmp_sur = find_surrounding(tmp_pos, tmp_t);
            bool do_not_break = false;
            // if(!tmp_sur[0].is_high_platform){
            // random_shuffle(tmp_sur.begin(),tmp_sur.end());
            //     mysort(tmp_sur,destination);

            // }else{
            //     do_not_break=true;
            // }
            mysort(tmp_sur, destination);

            for (uint j = 0; j < tmp_sur.size(); j++)
            {
                int add = 0;
                vector<Step> new_stps = tmp_stps;
                Step new_stp;
                float move = get_distance(tmp_pos, tmp_sur[j]);

                // if (get_distance(tmp_pos, destination) > get_distance(tmp_sur[j], destination)&&get_dely(tmp_pos,tmp_sur[j],tmp_t,0.115)<0.2)
                // if(get_dely(tmp_pos,tmp_sur[j],tmp_t,0.115)<0.2)
                if (true)
                {
                    // cout<<get_distance(tmp_pos, destination)<<endl;
                    // cout<<get_distance(tmp_sur[j], destination)<<endl;
                    if (tmp_sur[j].is_high_platform)
                    {
                        new_stp.type = "high";
                        new_stp.l = tmp_sur[j].id;
                        // new_stp.t = tmp_t +_tf+ _xishu * move + get_dely(tmp_pos, tmp_sur[j], tmp_t, 0.115);
                        new_stp.t = tmp_t + _tf + _xishu * move;
                    }
                    else
                    {
                        new_stp.m = tmp_sur[j].m;

                        new_stp.n = tmp_sur[j].n;
                        // cout<<new_stp.m<<","<<new_stp.n<<endl;

                        // new_stp.t = tmp_t +_tf+ _xishu * move + get_dely(tmp_pos, tmp_sur[j], tmp_t, 0.115);
                        new_stp.t = tmp_t + _tf + _xishu * move;
                        // cout<<new_stp.t<<endl;
                    }
                    new_stps.push_back(new_stp);
                    // if(visited.find(new_stp.l*10000+new_stp.m+100+new_stp.n)==visited.end()){
                    //     // Q.pop();
                    //     Q.push(new_stps);
                    //     visited.emplace(new_stp.l*10000+new_stp.m+100+new_stp.n,1);

                    // }else if(visited[new_stp.l*10000+new_stp.m+100+new_stp.n]==1){
                    //     Q.push(new_stps);
                    //     visited[new_stp.l*10000+new_stp.m+100+new_stp.n]=2;
                    // }
                    // is_path_used(tmp_pos,tmp_sur[j]);
                    bool used = false;
                    for (int k = 0; k < used_paths.size(); k++)
                    {
                        if (used_paths[k].pos1 == tmp_pos && used_paths[k].pos2 == tmp_sur[j])
                        {
                            used = true;
                        }
                        if (used_paths[k].pos2 == tmp_pos && used_paths[k].pos1 == tmp_sur[j])
                        {
                            used = true;
                        }
                    }
                    if (!used)
                    {
                        Q.push(new_stps);
                        Used_path used_path;
                        used_path.pos1 = tmp_pos;
                        used_path.pos2 = tmp_sur[j];
                        used_paths.push_back(used_path);
                    }

                    add++;
                    // if(Q.size()>100){
                    //     cout<<Q.size()<<endl;
                    // }

                    // if(add==1&&new_stps.size()%2==1&&new_stps.size()>4&&!do_not_break){
                    //     break;
                    // }
                    // if(add==2){
                    //     break;

                    // }
                }
            }
        }
    }

    stps = best_path;
    time_now = best_arive_time;
    cout << "time_now:" << time_now << endl;
}

void Drone::get_platforms(vector<High_Platform> _platforms)
{
    this->platforms = _platforms;
}

float Drone::get_dely(Position pos1, Position pos2, float start_time, float need_time)
{
    float result_dely = 0.0f;
    for (uint i = 0; i < dis_links.size(); i++)
    {
        if ((dis_links[i].pos1 == pos1 && dis_links[i].pos2 == pos2) || (dis_links[i].pos1 == pos2 && dis_links[i].pos2 == pos1))
        {
            if (start_time + need_time > dis_links[i].periods[0].start && start_time < dis_links[i].periods[0].end)
            {

                // return dis_links[i].periods[j].end - start_time;
                result_dely = dis_links[i].periods[0].end - start_time + get_dely(pos1, pos2, dis_links[i].periods[0].end, need_time);
            }
        }
    }
    // return 0.0f;
    return result_dely;
}

vector<Position> Drone::find_surrounding(Position pos, float time)
{
    vector<Position> ans;
    if (pos.is_base_station || pos.is_high_platform)
    {
        Position tmp_pos;
        for (uint i = 0; i < platforms.size(); i++)
        {
            if (pos.is_base_station)
            {
                tmp_pos = platforms[i].get_position();
                if (get_distance(pos, tmp_pos) < _drone_base)
                {
                    ans.push_back(tmp_pos);
                }
            }
            if (pos.id != platforms[i].id && pos.is_high_platform)
            {
                tmp_pos = platforms[i].get_position();
                if (get_distance(pos, tmp_pos) < _drone_drone)
                {
                    ans.push_back(tmp_pos);
                }
            }
        }
        int m = int((pos.x - _v * time) / _x_intraorbit);
        int n = int(pos.y / _x_intraorbit);
        vector<int> m_range, n_range;
        for (int i = 0; i < 5; i++)
        {
            m_range.push_back(m - 2 + i);
            n_range.push_back(n - 2 + i);
        }

        for (int i = 0; i < 5; i++)
        {
            for (int j = 0; j < 5; j++)
            {
                Position po1 = get_position_from_mn(m_range[i], n_range[j], time);
                float distan_1 = get_distance(po1, pos);
                if (pos.is_base_station)
                {
                    if (distan_1 < _drone_base)
                    // if (distan_1 < 2*_drone_drone)
                    {
                        ans.push_back(po1);
                    }
                }
                else
                {
                    if (distan_1 < _drone_drone)
                    {
                        ans.push_back(po1);
                    }
                }
            }
        }
    }
    if (pos.is_drone)
    {
        Position tmp_pos;
        for (uint i = 0; i < platforms.size(); i++)
        {
            tmp_pos = platforms[i].get_position();
            if (get_distance(pos, tmp_pos) < _drone_drone - 2)
            {
                ans.push_back(tmp_pos);
            }
        }
        Position po3 = get_position_from_mn(pos.m, pos.n - 1, time);
        ans.push_back(po3);
        Position po4 = get_position_from_mn(pos.m, pos.n + 1, time);
        ans.push_back(po4);
        Position po1 = get_position_from_mn(pos.m - 1, pos.n, time);
        ans.push_back(po1);
        Position po2 = get_position_from_mn(pos.m + 1, pos.n, time);
        ans.push_back(po2);
    }
    return ans;
}

void Drone::get_candidate_point(Position start, Position end, float star_time)
{
    useful_plat.resize(0);
    sta_mns.resize(0);
    des_mns.resize(0);
    path_points.resize(0);
    is_points_arive.resize(0);
    is_plat_used.resize(0);
    is_des_mn_used.resize(0);
    for (uint i = 0; i < platforms.size(); i++)
    {
        // float min_x = min(start.x, end.x) + 20;
        // float max_x = max(start.x, end.x) - 20;
        // float min_y = min(start.y, end.y) + 20;
        // float max_y = max(start.y, end.y) - 20;
        float min_x = min(start.x, end.x);
        float max_x = max(start.x, end.x);
        float min_y = min(start.y, end.y);
        float max_y = max(start.y, end.y);

        if (min_x < platforms[i].x && max_x > platforms[i].x && min_y < platforms[i].y && max_y > platforms[i].y)
        {
            useful_plat.push_back(platforms[i]);
            is_plat_used.push_back(false);
        }
    }
    sta_mns = find_surrounding(start, star_time);
    float x_distance = fabs(start.x - end.x);
    float y_distance = fabs(start.y - end.y);
    // float tmp_y_distance;
    // tmp_y_distance = y_distance - floor(y_distance / _y_interorbit) * _y_interorbit;
    // float tmp_x_distance;
    // tmp_x_distance = x_distance - floor(x_distance / _x_intraorbit) * _x_intraorbit;
    float arriving_time = star_time + 0.1 * (floor(x_distance / _x_intraorbit) + floor(y_distance / _y_interorbit) + 1) + _xishu * (x_distance + y_distance);
    // des_mns = find_surrounding(end, arriving_time);
    int m = int((end.x - _v * arriving_time) / _x_intraorbit);
    int n = int(end.y / _x_intraorbit);
    vector<int> m_range, n_range;
    for (int i = 0; i < 5; i++)
    {
        m_range.push_back(m - 2 + i);
        n_range.push_back(n - 2 + i);
    }

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            Position po1 = get_position_from_mn(m_range[i], n_range[j], arriving_time);
            float distan_1 = get_distance(po1, end);

            // if (distan_1 < 4*_drone_drone)
            // // if (distan_1 < 2*_drone_drone)
            // {
            //     des_mns.push_back(po1);
            // }
            des_mns.push_back(po1);
        }
    }

    for (uint i = 0; i < des_mns.size(); i++)
    {
        is_des_mn_used.push_back(false);
    }
}

void Drone::reset()
{
    for (uint i = 0; i < is_points_arive.size(); i++)
    {
        is_points_arive[i] = false;
    }
}
void Drone::find_step(
    Step &stp,
    float &time_now,
    Position &position_now,
    Position destination,
    bool &finish,
    int sta_mn_id,
    int plt_id1,
    int plt_id2,
    int des_mn_id)
{
    if (position_now.is_base_station)
    {
        path_points.resize(0);
        is_points_arive.resize(0);

        path_points.push_back(sta_mns[sta_mn_id]);
        is_points_arive.push_back(false);
        if (plt_id1 < useful_plat.size())
        {
            path_points.push_back(useful_plat[plt_id1].get_position());
            is_points_arive.push_back(false);
        }
        if (plt_id2 < useful_plat.size())
        {
            path_points.push_back(useful_plat[plt_id2].get_position());
            is_points_arive.push_back(false);
        }

        if (des_mn_id < des_mns.size())
        {
            path_points.push_back(des_mns[des_mn_id]);
            is_points_arive.push_back(false);
        }
        path_points.push_back(destination);
        is_points_arive.push_back(false);
    }
    Position start_position = position_now;
    float start_time = time_now;
    if (is_distance_suit(position_now, destination, time_now))
    {
        finish = true;
        return;
    }
    vector<Position> surrounds = find_surrounding(position_now, time_now);
    Position tmp_des;
    int tmp_des_id = -1;
    for (int i = 0; i < is_points_arive.size(); i++)
    {
        if (!is_points_arive[i])
        {
            tmp_des = path_points[i];
            tmp_des_id = i;
            break;
        }
    }
    if (tmp_des.is_drone)
    {
        tmp_des = get_position_from_mn(tmp_des.m, tmp_des.n, time_now);
    }
    Position find;
    int best_id = -1;
    float best_dis = 10000.0f;
    float dely_tmp_des=get_dely(start_position,tmp_des,start_time,0.12);
    float dis_to_des=get_distance(start_position, tmp_des)+5*(dely_tmp_des+0.12);
    if (!start_position.is_base_station && dis_to_des < _drone_drone && !tmp_des.is_base_station)
    {
        find = tmp_des;
        is_points_arive[tmp_des_id] = true;
        // best_dis = get_distance(start_position, tmp_des);
    }
    else if (start_position.is_base_station && dis_to_des < 70)
    {
        find = tmp_des;
        is_points_arive[tmp_des_id] = true;
        // best_dis = get_distance(start_position, tmp_des);
    }
    else
    {

        for (uint i = 0; i < surrounds.size(); i++)
        {
            float need_move = get_distance(start_position, surrounds[i]);
            if (start_position.is_base_station && need_move > _drone_base)
            {
                continue;
            }
            if (!start_position.is_base_station && need_move > _drone_drone - 2)
            {
                continue;
            }
            float tmp_dis = get_distance(surrounds[i], tmp_des);

            // if(start_position.is_high_platform||surrounds[i].is_high_platform){

            //         continue;

            // }
            if (!start_position.is_base_station && need_move < _drone_drone && tmp_dis < best_dis)
            {
                best_dis = tmp_dis;
                best_id = i;
            }
            if (start_position.is_base_station && need_move < _drone_base && tmp_dis < best_dis)
            {
                best_dis = tmp_dis;
                best_id = i;
            }
        }
        find = surrounds[best_id];
        // cout << find.x << "," << find.y << endl;
    }
    float move_dis = get_distance(start_position, find);
    Position new_start=start_position,new_find=find;
    float new_move_dis;
    float using_time=get_dely(start_position, find, start_time, 0.12)+_tf+_xishu*move_dis;
    if(start_position.is_drone){
        new_start=get_position_from_mn(start_position.m,start_position.n,start_time+using_time);
    }
    if(find.is_drone){
        new_find=get_position_from_mn(find.m,find.n,start_time+using_time);
    }
    new_move_dis=get_distance(new_start,new_find);
    if(!start_position.is_base_station){
        move_dis=max(move_dis,new_move_dis);

    }
    if(start_position.is_drone&&find.is_drone){
        move_dis=abs(start_position.m-find.m)*_x_intraorbit+abs(start_position.n-find.n)*_y_interorbit+0.01;
    }
    // if(!start_position.is_drone||!find.is_drone){
    //     move_dis += 5 * get_dely(start_position, find, start_time, 0.11)+0.6;
    // }
    
    // if (start_position.is_high_platform || find.is_high_platform)
    // {
    //     move_dis += 1;
    // }

    // if(move_dis>115){
    //     cout<<move_dis<<endl;
    // }
    time_now += _tf + _xishu * move_dis + get_dely(start_position, find, start_time, 0.11);
    // time_now += _tf + _xishu * move_dis;
    // Disable_link tmp_link;
    // Period tmp_per;
    // tmp_per.start = start_time;
    // tmp_per.end = time_now;
    // tmp_link.periods.push_back(tmp_per);
    // tmp_link.pos1 = start_position;
    // tmp_link.pos2 = find;
    // cout<<start_position.is_base_station<<endl;
    // cout<<position_now.is_base_station<<endl;
    // dis_links.push_back(tmp_link);
    stp.m = find.m;
    stp.n = find.n;
    // if ( start_position.is_high_platform || find.is_high_platform)
    // {
    //     time_now -= 0.05;
    // }
    if (find.is_high_platform)
    {
        stp.type = "high";
        stp.l = find.id;
        position_now = find;
    }
    else
    {
        position_now = get_position_from_mn(find.m, find.n, time_now);
    }
    stp.t = time_now;
    // position_now = find;
}

void Drone::find_best_drone(
    Step &stp,
    float &time_now,
    Position &position_now,
    Position destination,
    bool &finish)
{
    Position start_position = position_now;
    float start_time = time_now;
    // vector<Position> suround = find_surrounding(position_now, time_now);
    // cout << suround.size() << endl;

    // find the best drone for base station
    if (position_now.is_base_station)
    {
        // cout<<find_surrounding(platforms[1].get_position(),time_now).size()<<endl;
        int m = int((position_now.x - _v * time_now) / _x_intraorbit);
        int n = int(position_now.y / _x_intraorbit);
        vector<int> m_range, n_range;
        for (int i = 0; i < 5; i++)
        {
            m_range.push_back(m - 2 + i);
            n_range.push_back(n - 2 + i);
        }

        float sta_to_end = get_distance(position_now, destination);

        float best_distance = sta_to_end + 500;
        int best_m = INT32_MAX;
        int best_n = INT32_MAX;
        float best_dely = 10.0f;
        for (int i = 0; i < 5; i++)
        {
            for (int j = 0; j < 5; j++)
            {
                Position po1 = get_position_from_mn(m_range[i], n_range[j], time_now);
                float distan_1 = get_distance(po1, position_now);
                Position po2 = get_position_from_mn(m_range[i], n_range[j], time_now + _tf + distan_1 * _xishu);
                float distan_2 = get_distance(po2, position_now);
                float dis_to_destination = get_distance(po1, destination);
                float dely = get_dely(start_position, po1, time_now, 0.11);

                dis_to_destination += dely * 800;
                if (max(distan_1, distan_2) < _drone_base && dis_to_destination < best_distance)
                // if (max(distan_1, distan_2) < _drone_base && dely < best_dely && dis_to_destination < best_distance)
                // if (max(distan_1, distan_2) < _drone_base && distan_1 < best_distance)
                {
                    // best_distance =distan_1;

                    best_distance = dis_to_destination;
                    // best_dely = dely;
                    // cout<<"best_dely"<<best_dely<<endl;
                    best_m = m_range[i];
                    best_n = n_range[j];
                }
            }
        }
        position_now.m = best_m;
        position_now.n = best_n;
        // cout<<best_m<<endl;
        // cout<<best_n<<endl;
        Position tmp = get_position_from_mn(best_m, best_n, time_now);
        // Position tmp1=get_position_from_mn(best_m, best_n, time_now+1.9);
        // if(tmp==tmp1){
        //     cout<<"useful"<<endl;
        // }

        // float tmp_dis = get_distance(tmp, position_now);
        // tmp_dis = floor(tmp_dis);

        position_now.is_base_station = false;
        position_now.is_drone = true;
        time_now += get_dely(start_position, tmp, time_now, 0.11);
        this->y_first = !y_first;

        Position po_1 = get_position_from_mn(best_m, best_n, time_now);
        Position po_2 = get_position_from_mn(best_m, best_n, time_now + _tf + get_distance(start_position, po_1) * _xishu);

        float x_distance = fabs(po_2.x - destination.x);
        float y_distance = fabs(po_2.y - destination.y);
        float tmp_y_distance;
        tmp_y_distance = y_distance - floor(y_distance / _y_interorbit) * _y_interorbit;
        float tmp_x_distance;
        tmp_x_distance = x_distance - floor(x_distance / _x_intraorbit) * _x_intraorbit;
        float arriving_time = time_now + 0.1 * (floor(x_distance / _x_intraorbit) + floor(y_distance / _y_interorbit) + 1) + _xishu * (x_distance + y_distance);
        // cout << find_surrounding(destination, arriving_time).size() << endl;
        // cout << "expectide arriving time:" << arriving_time << endl;

        if (get_distance(start_position, po_1) >= 70.0 || get_distance(start_position, po_2) >= 70.0)
        {
            cout << "wrong" << endl;
        }
        //     float time=is_link_avilable(start_position,tmp,time_now);
        //    cout<<"time"<<time<<endl;
        //     time_now+=time;
    }
    else
    {
        if (is_distance_suit(position_now, destination, time_now))

        {
            finish = true;
        }
        else
        {

            float x_distance = fabs(position_now.x - destination.x);
            float y_distance = fabs(position_now.y - destination.y);
            int tmp_m, tmp_n;
            bool move = false;
            if (y_first)
            {
                // this->y_first = !y_first;
                if (y_distance > 53)
                // if (y_distance > _y_interorbit/2)
                {
                    if (position_now.y < destination.y)
                    {
                        tmp_n = position_now.n + 1;
                    }
                    else
                    {
                        tmp_n = position_now.n - 1;
                    }
                    Position tmp = get_position_from_mn(position_now.m, tmp_n, time_now);

                    time_now += get_dely(start_position, tmp, time_now, _tf + _xishu * _y_interorbit);
                    position_now.n = tmp_n;
                    move = true;
                }
                // if (x_distance > _drone_base)
                if (!move && x_distance > _x_intraorbit / 2)
                {
                    // this->y_first = !y_first;
                    if (position_now.x < destination.x)
                    {
                        // position_now.m += 1;
                        tmp_m = position_now.m + 1;
                    }
                    else
                    {
                        // position_now.m -= 1;
                        tmp_m = position_now.m - 1;
                    }
                    Position tmp = get_position_from_mn(tmp_m, position_now.n, time_now);
                    time_now += get_dely(start_position, tmp, time_now, _tf + _x_intraorbit * _xishu);
                    move = true;
                    position_now.m = tmp_m;
                }
            }

            else
            {
                // this->y_first = !y_first;

                // if (x_distance > 50)
                // if (x_distance > 40)
                if (x_distance > _x_intraorbit / 2)
                {
                    if (position_now.x < destination.x)
                    {

                        tmp_m = position_now.m + 1;
                    }
                    else
                    {

                        tmp_m = position_now.m - 1;
                    }
                    Position tmp = get_position_from_mn(tmp_m, position_now.n, time_now);

                    time_now += get_dely(start_position, tmp, time_now, _tf + _xishu * _y_interorbit);
                    position_now.m = tmp_m;
                    move = true;
                }
                // if (x_distance > _drone_base)
                if (!move && y_distance > _y_interorbit / 2)
                {
                    // this->y_first = !y_first;
                    if (position_now.y < destination.y)
                    {
                        tmp_n = position_now.n + 1;
                    }
                    else
                    {
                        tmp_n = position_now.n - 1;
                    }
                    Position tmp = get_position_from_mn(position_now.m, tmp_n, time_now);
                    // cout<<get_dely(start_position, tmp, time_now, _tf + _xishu * _y_interorbit)<<endl;

                    time_now += get_dely(start_position, tmp, time_now, _tf + _xishu * _y_interorbit);
                    position_now.n = tmp_n;
                }
            }
        }
    }
    if (!finish)
    {
        stp.m = position_now.m;
        stp.n = position_now.n;
        Position tmp_pos = get_position_from_mn(position_now.m, position_now.n, time_now);
        float dis = get_distance(start_position, tmp_pos);

        time_now += _tf + _xishu * dis;
        Position xyz = get_position_from_mn(position_now.m, position_now.n, time_now);

        position_now.x = xyz.x;
        position_now.y = xyz.y;
        position_now.z = xyz.z;
        position_now.is_drone = true;

        Disable_link tmp_link;
        Period tmp_per;
        tmp_per.start = start_time;
        tmp_per.end = time_now;
        tmp_link.periods.push_back(tmp_per);
        tmp_link.pos1 = start_position;
        tmp_link.pos2 = position_now;
        // cout<<start_position.is_base_station<<endl;
        // cout<<position_now.is_base_station<<endl;
        dis_links.push_back(tmp_link);
        stp.t = time_now;
    }
    else
    {
        Disable_link tmp_link;
        Period tmp_per;
        tmp_per.start = start_time;
        tmp_per.end = time_now;
        tmp_link.periods.push_back(tmp_per);
        tmp_link.pos1 = start_position;
        tmp_link.pos2 = destination;
        // cout<<start_position.is_base_station<<endl;
        // cout<<position_now.is_base_station<<endl;
        dis_links.push_back(tmp_link);
        stp.t = time_now;
    }
}

float Drone::is_link_avilable(Position pos1, Position pos2, float time_now)
{
    // cout<<dis_links.size()<<endl;
    // float max_time=0.0f;
    for (uint i = 0; i < dis_links.size(); i++)
    {
        if ((dis_links[i].pos1 == pos1 && dis_links[i].pos2 == pos2) || (dis_links[i].pos1 == pos2 && dis_links[i].pos2 == pos1))
        // if (dis_links[i].pos1 == pos1 && dis_links[i].pos2 == pos2)
        {

            for (uint j = 0; j < dis_links[i].periods.size(); j++)
            {
                if (time_now + 0.109 > dis_links[i].periods[j].start && time_now < dis_links[i].periods[j].end)
                {

                    return dis_links[i].periods[j].end - time_now;
                }
            }
        }
    }
    return 0.0f;
    // return time_now;
}

float Drone::get_distance(Position pos1, Position pos2)
{
    float dis, dis_2;
    dis_2 = pow((pos2.x - pos1.x), 2) + pow((pos2.y - pos1.y), 2) + pow((pos2.z - pos1.z), 2);
    dis = sqrt(dis_2);
   
    // dis = floor(dis);
    return dis;
}

Position Drone::get_position_from_mn(int _m, int _n, float time_now)
{
    Position posi;
    posi.x = _v * time_now + _m * _x_intraorbit;
    posi.y = _n * _y_interorbit;
    posi.z = _H;
    posi.m = _m;
    posi.n = _n;
    posi.is_drone = true;
    return posi;
}

bool Drone::is_distance_suit(Position pos1, Position pos2, float &time_now)
{
    if (pos1.is_base_station)
    {
        return false;
    }
    if (pos1.is_high_platform)
    {
        float distance = get_distance(pos1, pos2);
        if (distance < _drone_base)
        {
            float high_dely = get_dely(pos1, pos2, time_now, _tf + _xishu * distance);
            time_now = time_now + high_dely + _tf + _xishu * distance;
            return true;
        }
        else
        {
            return false;
        }
    }
    if (pos1.is_drone)
    {
        float distance1, distance2;

        distance1 = get_distance(pos1, pos2);
        // cout<<distance1<<endl;
        float dely = get_dely(pos1, pos2, time_now, _tf + _xishu * distance1);
        // float dely=0.0f;
        Position tmp_pos1 = get_position_from_mn(pos1.m, pos1.n, time_now + dely + _tf + _xishu * distance1);
        distance2 = get_distance(tmp_pos1, pos2);

        if (distance1 < _drone_base && distance2 < _drone_base)
        {

            time_now += _tf + _xishu * max(distance1, distance2) + dely;
            // time_now += 0.0001;
            // cout<<"distance1: "<<distance1<<endl;
            // cout<<"distance2: "<<distance2<<endl;
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

Drone::~Drone()
{
}
class Generator
{
private:
    string _Output = "result.txt";
    ofstream ofs;
    int s = 10;
    int c = 3;
    vector<Base_Station> base_stations;
    vector<High_Platform> platforms;
    vector<Demand> demans;
    vector<Solution> solutions;
    vector<vector<int>> location = {{-614, 1059, 24}, {-934, 715, 12}, {1073, 291, 37}, {715, 129, 35}, {186, 432, 21}, {-923, 632, 37}, {833, 187, 24}, {-63, 363, 11}};

public:
    /**
     * @brief Construct a new Generator object
     *
     * @param Output
     */
    Generator(string Output);

    /**
     * @brief
     *
     */
    void do_generate();

    /**
     * @brief Get the base station object
     *
     */
    void get_base_station();

    void get_platform();

    /**
     * @brief Get the demand object
     *
     */
    void get_demand();

    /**
     * @brief
     *
     */
    void generate_result();

    /**
     * @brief
     *
     */
    void output();

    void output_mn();
    void output_to_terminal();

    /**
     * @brief Destroy the Generator object
     *
     */
    ~Generator();
};

Generator::Generator(string Output)
{
    this->_Output = Output;
}

void Generator::do_generate()
{
    get_base_station();
    get_platform();
    get_demand();

    generate_result();

    output();
    // output_to_terminal();
    // output_mn();
}

void Generator::get_base_station()
{
    Base_Station sta0, sta1, sta2;
    sta0.init(0, 45.73, 45.26, 0.0);
    sta1.init(1, 1200, 700, 0.0);
    sta2.init(2, -940, 1100, 0.0);

    base_stations.push_back(sta0);
    base_stations.push_back(sta1);
    base_stations.push_back(sta2);
}

void Generator::get_platform()
{
    for (uint i = 0; i < location.size(); i++)
    {
        High_Platform plat;
        plat.init(location[i], i);
        this->platforms.push_back(plat);
    }
}

void Generator::get_demand()
{
    float t[] = {0, 4.7, 16.4};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                if (j != k)
                {
                    int tmp_s = s;
                    float tmp_t = t[i];
                    while (tmp_s > 0)
                    {
                        Demand dem1;
                        if (tmp_s >= c)
                        {
                            dem1.init(tmp_t, base_stations[j], base_stations[k], c);
                            tmp_s -= c;
                        }
                        else
                        {
                            dem1.init(t[i], base_stations[j], base_stations[k], tmp_s);
                            tmp_s = 0;
                        }

                        this->demans.push_back(dem1);
                    }
                }
            }
        }
    }
}

void Generator::generate_result()
{

    Drone drone;
    drone.get_platforms(this->platforms);
    int last_stat_id = -1;
    int last_end_id = -1;
    for (Demand dem : demans)
    {
        Solution sol;

        sol.time = dem.time;
        sol.start_id = dem.sta_station.id;
        sol.end_id = dem.end_station.id;
        sol.num_signals = dem.num_signals;
        if (last_stat_id != sol.start_id || last_end_id != sol.end_id)
        {
            drone.dis_links.resize(0);

            // cout<<"did something"<<endl;
        }

        // float time_now = dem.time;
        Position position_now, destination;
        position_now.init(true, dem.sta_station.x, dem.sta_station.y, dem.sta_station.z);
        position_now.id = dem.sta_station.id;

        destination.init(true, dem.end_station.x, dem.end_station.y, dem.end_station.z);
        destination.id = dem.end_station.id;
        if (last_stat_id != sol.start_id || last_end_id != sol.end_id)
        {
            drone.get_candidate_point(position_now, destination, dem.time);
            // cout<<drone.useful_plat.size()<<"!"<<endl;

            // cout<<drone.des_mns.size()<<".."<<endl;
        }
        // drone.reset();
        float best_dely = 1000.0f;
        vector<Step> best_steps;
        // drone.search_by_bfs(best_steps,time_now,position_now,destination);
        for (uint s = 0; s < drone.sta_mns.size(); s++)
            for (uint i = 0; i < drone.useful_plat.size() + 1; i++)
            {
                for (int k = 0; k < drone.useful_plat.size() + 1; k++)
                {
                    // if (i != k)
                    if (true)
                    {
                        for (uint j = 0; j < drone.des_mns.size() + 1; j++)
                        {
                            float time_now = dem.time;
                            vector<Step> tmp_stp;
                            Position start_pos = position_now;
                            bool finish = false;
                            // cout<<i<<","<<j<<endl;
                            while (!finish)
                            {
                                // cout<<"!!"<<endl;
                                Step stp;

                                // drone.find_best_drone(stp, time_now, position_now, destination, finish);
                                drone.find_step(stp, time_now, start_pos, destination, finish, s, i, k, j);

                                if (!finish)
                                {
                                    tmp_stp.push_back(stp);
                                }
                            }
                            if (time_now - dem.time < best_dely)
                            {
                                best_dely = time_now - dem.time;
                                best_steps = tmp_stp;
                                // cout<<"!!!"<<endl;
                                // cout<<best_steps.size()<<endl;
                                // cout<<best_dely<<endl;
                            }
                        }
                    }
                }
            }
        // cout<<best_dely<<endl;
        sol.delay = best_dely;
        sol.steps = best_steps;
        float last_time = dem.time;
        Position last_pos = position_now;
        for (uint i = 0; i < best_steps.size(); i++)
        {
            Disable_link tmp_link;
            Period tmp_per;
            tmp_per.start = last_time;
            tmp_per.end = best_steps[i].t;
            last_time = best_steps[i].t;
            tmp_link.periods.push_back(tmp_per);
            tmp_link.pos1 = last_pos;
            if (best_steps[i].type == "high")
            {
                // tmp_link.pos2 = drone.platforms[i].get_position();
                tmp_link.pos2 = drone.platforms[best_steps[i].l].get_position();
                last_pos = tmp_link.pos2;
            }
            else
            {
                tmp_link.pos2 = drone.get_position_from_mn(best_steps[i].m, best_steps[i].n, best_steps[i].t);
                last_pos = tmp_link.pos2;
            }
            drone.dis_links.push_back(tmp_link);
        }
        Disable_link tmp_link;
        Period tmp_per;
        tmp_per.start = last_time;
        tmp_per.end = sol.time + sol.delay;
        // cout<<"end_time"<<tmp_per.end<<endl;
        tmp_link.periods.push_back(tmp_per);
        tmp_link.pos1 = last_pos;
        tmp_link.pos2 = destination;
        drone.dis_links.push_back(tmp_link);

        // while (!finish)
        // {
        //     // cout<<"!!"<<endl;
        //     Step stp;
        //     // drone.find_best_drone(stp, time_now, position_now, destination, finish);
        //     drone.find_step(stp, time_now, position_now, destination, finish);

        //     if (!finish)
        //     {
        //         sol.steps.push_back(stp);
        //     }
        // }

        // sol.delay = time_now - dem.time;
        this->solutions.push_back(sol);

        last_stat_id = sol.start_id;
        last_end_id = sol.end_id;
    }
}

void Generator::output()
{

    ofs.open(_Output, ios::trunc);
    float total = 0.0f;
    for (Solution sol : solutions)
    {
        total += sol.delay * sol.num_signals;
        ofs << setiosflags(ios::fixed) << setprecision(4) << sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay << "," << sol.num_signals << endl;

        for (uint i = 0; i < sol.steps.size(); i++)
        {
            if (i != 0)
            {
                ofs << ",";
            }
            if (sol.steps[i].type == "high")
            {
                ofs << "(" << sol.steps[i].t << "," << sol.steps[i].l << ")";
            }
            else
            {
                ofs << "(" << sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
            }
        }
        if (sol.steps.size() != 0)
        {
            ofs << endl;
        }
    }
    // ofs << "total:" << total;
    // cout << "total:" << total << endl;

    ofs.close();
}

void Generator::output_mn()
{
    ofs.open("mn.txt", ios::trunc);
    for (Solution sol : solutions)
    {

        for (uint i = 0; i < sol.steps.size(); i++)
        {

            ofs << sol.steps[i].m << "," << sol.steps[i].n << endl;
        }
    }
    ofs.close();
}

void Generator::output_to_terminal()
{
    for (Solution sol : solutions)
    {
        cout << setiosflags(ios::fixed) << setprecision(4) << sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay << "," << sol.num_signals << endl;

        for (uint i = 0; i < sol.steps.size(); i++)
        {
            if (i != 0)
            {
                cout << ",";
            }
            if (sol.steps[i].type == "high")
            {
                cout << "(" << sol.steps[i].t << "," << sol.steps[i].l << ")";
            }
            else
            {
                cout << "(" << sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
            }
            // cout << "(" << setiosflags(ios::fixed) << setprecision(4) << sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
        }
        if (sol.steps.size() != 0)
        {
            cout << endl;
        }
    }
}

Generator::~Generator()
{
}

string Output = "result.txt";
int main()
{
    Generator generator(Output);
    generator.do_generate();
    generator.output_to_terminal();
    return 0;
}