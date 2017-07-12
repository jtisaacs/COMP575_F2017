#include "Saturation.h"

float Saturation::applyMaximumLimit(float independent_variable, float maximum_value_limit)
{
    return independent_variable > maximum_value_limit ? maximum_value_limit : independent_variable;
}

float Saturation::applyMinimumLimit(float independent_variable, float minimum_value_limit)
{
    return independent_variable < minimum_value_limit ? minimum_value_limit : independent_variable;
}

float Saturation::saturation(float independent_variable, float maximum_absolute_value)
{
    independent_variable = applyMaximumLimit(independent_variable,maximum_absolute_value);
    independent_variable = applyMinimumLimit(independent_variable,-maximum_absolute_value);
    return independent_variable;
}
