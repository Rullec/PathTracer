#include "cTimeUtil.hpp"
#include <ctime>
#include <ratio>
#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono;

static high_resolution_clock::time_point t1, t2;
void cTimeUtil::Begin()
{
    t1 = high_resolution_clock::now();
}

void cTimeUtil::End()
{
    t2 = high_resolution_clock::now();
    std::cout <<"cTimeUtil: cost time = " << (t2 - t1).count() * 1e-6 <<" ms\n";
}