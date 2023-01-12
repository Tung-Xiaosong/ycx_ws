#ifndef _SIMULATOR_PP_H_
#define _SIMULATOR_PP_H_
namespace my_simulator{
class Calculator{
public:
    virtual double operate() = 0;
    virtual void init(double a, double b) = 0;
};
}

#endif
