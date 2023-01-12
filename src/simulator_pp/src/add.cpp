#include <simulator_pp/add.h>
#include <pluginlib/class_list_macros.h>

namespace sim_add
{
void Add::init(double aa,  double bb){
    a = aa;
    b = bb;
}
double Add::operate(){
    return a+b;
}
} // namespace sim
PLUGINLIB_EXPORT_CLASS(sim_add::Add, my_simulator::Calculator);
