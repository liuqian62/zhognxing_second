#include "Generator.h"

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
        }        
        float best_dely = 1000.0f;
        vector<Step> best_steps;        
        for(uint s=0;s<drone.sta_mns.size();s++)
        for (uint i = 0; i < drone.useful_plat.size() + 1; i++)
        {
            for (int k = 0; k < drone.useful_plat.size() + 1; k++)
            {                
                if(true)
                {
                    for (uint j = 0; j < drone.des_mns.size() + 1; j++)
                    {
                        float time_now = dem.time;
                        vector<Step> tmp_stp;
                        Position start_pos = position_now;
                        bool finish = false;                        
                        while (!finish)
                        {                            
                            Step stp;                            
                            drone.find_step(stp, time_now, start_pos, destination, finish, s,i, k, j);

                            if (!finish)
                            {
                                tmp_stp.push_back(stp);
                            }
                        }
                        if (time_now - dem.time < best_dely)
                        {
                            best_dely = time_now - dem.time;
                            best_steps = tmp_stp;                            
                        }
                    }
                }
            }
        }        
        sol.delay = best_dely;
        sol.steps = best_steps;
        cout<<drone.des_mns.size()<<endl;
        // cout<<best_steps.size()<<endl;
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
                tmp_link.pos2 = drone.platforms[best_steps[i].l].get_position();
                // cout<<drone.get_distance(last_pos,tmp_link.pos2)<<endl;
                        
                last_pos = tmp_link.pos2;
                // cout<<last_time<<endl;
                // cout<<best_steps[i].l<<endl;
                // cout<<tmp_link.pos2.x<<","<<tmp_link.pos2.y<<endl;
            }
            else
            {
                tmp_link.pos2 = drone.get_position_from_mn(best_steps[i].m, best_steps[i].n, best_steps[i].t);
                last_pos = tmp_link.pos2;                
            }
            drone.dis_links.push_back(tmp_link);
            // if(best_steps[i].type=="high"){
            //     cout<<drone.get_dely(tmp_link.pos1,tmp_link.pos2,tmp_per.end-0.005,0.4)<<endl;
            // }
        }
        Disable_link tmp_link;
        Period tmp_per;
        tmp_per.start = last_time;
        tmp_per.end = sol.time + sol.delay;        
        tmp_link.periods.push_back(tmp_per);
        tmp_link.pos1 = last_pos;
        tmp_link.pos2 = destination;        
        drone.dis_links.push_back(tmp_link); 
        // cout<<drone.get_dely(last_pos,destination,tmp_per.end-0.005,0.115)<<endl;       
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
    ofs << "total:" << total;
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