#ifndef SATURATION_H
#define SATURATION_H

class Saturation
{

private:

static float applyMaximumLimit(float independent_variable, float maximum_value_limit);
static float applyMinimumLimit(float independent_variable, float minimum_value_limit);

public:

static float saturation(float independent_variable, float maximum_absolute_value);

};
#endif // SATURATION_H
