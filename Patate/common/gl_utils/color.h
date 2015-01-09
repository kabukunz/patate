#ifndef _PATATE_COMMON_GL_UTILS_COLOR_
#define _PATATE_COMMON_GL_UTILS_COLOR_


#include "Eigen/Dense"


namespace PatateCommon
{


inline Eigen::Vector4f linearToSrgb(const Eigen::Vector4f& linear)
{
    Eigen::Vector4f srgb = linear;
    for(int i=0; i<3; ++i)
        srgb(i) = linear(i) > 0.0031308?
                    1.055 * std::pow(linear(i), 1/2.4):
                    12.92 * linear(i);
    return srgb;
}


inline Eigen::Vector4f srgbToLinear(const Eigen::Vector4f& srgb)
{
    Eigen::Vector4f linear = srgb;
    for(int i=0; i<3; ++i)
        linear(i) = linear(i) > 0.04045?
                    std::pow((linear(i)+0.055) / 1.055, 2.4):
                    linear(i) / 12.92;
    return linear;
}


}


#endif
