
#include <string>
#include <iostream>
#include "Generator.h"

using namespace std;

string Output = "result.txt";

int main()
{
    Generator generator(Output);
    generator.do_generate();
    generator.output_to_terminal();
    return 0;
}