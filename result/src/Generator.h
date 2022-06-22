#ifndef __GENERATOR_H__
#define __GENERATOR_H__
#include <string>
#include <fstream>
#include "utils.hpp"
#include "Drone.h"
#include <iostream>
#include<iomanip>

// #include <vector>
// #include <algorithm>
using namespace std;

class Generator
{
private:
    string _Output = "result.txt";
    ofstream ofs;
    int s=10;
    int c=3;
    vector<Base_Station> base_stations;
    vector<High_Platform> platforms;
    vector<Demand> demans;
    vector<Solution> solutions;
    vector<vector<int>> location = {{-614,1059,24},{-934,715,12},{1073,291,37},{715,129,35},{186,432,21},{-923,632,37},{833,187,24},{-63,363,11}};

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

#endif