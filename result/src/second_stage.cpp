
#include <string>
#include <iostream>
#include "Generator.h"

using namespace std;

string Output = "result.txt";

int main()
{
    Generator generator(Output);
    generator.do_generate();
    return 0;
}