#ifndef _SIMULATOR_ADD_H_
#define _SIMULATOR_ADD_H_
#include "simulator.h"
namespace sim_add
{
class Add: public my_simulator::Calculator{
public:
    void init(double aa, double bb);
    double operate();
private:
    double a, b;
};
} // namespace sim_add


#endif
