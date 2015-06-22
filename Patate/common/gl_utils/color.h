/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _PATATE_COMMON_GL_UTILS_COLOR_
#define _PATATE_COMMON_GL_UTILS_COLOR_


#include "Eigen/Dense"


namespace PatateCommon
{


enum ColorSpace {
    COLOR_NONE,
    COLOR_SRGB,
    COLOR_LINEAR_RGB,
    COLOR_CIE_XYZ,
    COLOR_CIE_LAB
};


inline Eigen::Vector4f linearRGBToSrgb(const Eigen::Vector4f& linear)
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


inline Eigen::Vector3f srgbFromLinearRGB(const Eigen::Vector3f& lrgb)
{
    Eigen::Vector3f srgb = lrgb;
    for(int i=0; i<3; ++i)
        srgb(i) = lrgb(i) > 0.0031308?
                    1.055 * std::pow(lrgb(i), 1/2.4):
                    12.92 * lrgb(i);
    return srgb;
}


inline Eigen::Vector3f linearRGBFromSrgb(const Eigen::Vector3f& srgb)
{
    Eigen::Vector3f lrgb = srgb;
    for(int i=0; i<3; ++i)
        lrgb(i) = lrgb(i) > 0.04045?
                    std::pow((lrgb(i)+0.055) / 1.055, 2.4):
                    lrgb(i) / 12.92;
    return lrgb;
}


inline Eigen::Vector3f linearRGBFromCieXYZ(const Eigen::Vector3f& cieXYZ) {
    static const Eigen::Matrix3f xyzToRgb = (Eigen::Matrix3f() <<
         3.2406, -1.5372, -0.4986,
        -0.9689,  1.8758,  0.0415,
         0.0557, -0.2040,  1.0570
    ).finished();

    return xyzToRgb * cieXYZ;
}


inline Eigen::Vector3f cieXYZFromLinearRGB(const Eigen::Vector3f& lrgb) {
    static const Eigen::Matrix3f rgbToxyz = (Eigen::Matrix3f() <<
         0.4124, 0.3576, 0.1805,
         0.2126, 0.7152, 0.0722,
         0.0193, 0.1192, 0.9505
    ).finished();

    return rgbToxyz * lrgb;
}


namespace internal {

    template < typename Scalar >
    struct LabHelper {
    static const Scalar thresold  = 6. / 29.;
    static const Scalar thresold2 = thresold  * thresold;
    static const Scalar thresold3 = thresold2 * thresold;
    static const Eigen::Matrix<Scalar, 3, 1> white;

    static inline Scalar f(Scalar v) {
        return (v > thresold3)?
                    std::pow(v, 1./3.):
                    1. / (3. * thresold2) * v + (4. / 29.);
    }

    static inline Scalar fInv(Scalar v) {
        return (v > thresold)?
                    std::pow(v, 3.):
                    3. * thresold2 * (v - (4. / 29.));
    }
    };

    template < typename Scalar >
    const Eigen::Matrix<Scalar, 3, 1> LabHelper<Scalar>::white =
            Eigen::Matrix<Scalar, 3, 1>(0.95047, 1, 1.08883);
}


inline Eigen::Vector3f cieLabFromCieXYZ(const Eigen::Vector3f& cieXYZ)
{
    typedef internal::LabHelper<float> LH;

    float fy = LH::f(cieXYZ(1) / LH::white(1));
    return Eigen::Vector3f(
                1.16 * fy - 0.16,
                5.00 * (LH::f(cieXYZ(0) / LH::white(0)) - fy),
                2.00 * (fy - LH::f(cieXYZ(2) / LH::white(2))));
}


inline Eigen::Vector3f cieXYZFromCieLab(const Eigen::Vector3f& cielab)
{
    typedef internal::LabHelper<float> LH;

    float lf = (cielab(0) + 0.16) / 1.16;
    return Eigen::Vector3f(
                LH::white(0) * LH::fInv(lf + cielab(1) / 5.00),
                LH::white(1) * LH::fInv(lf),
                LH::white(2) * LH::fInv(lf - cielab(2) / 2.00));
}


inline ColorSpace colorSpaceFromName(const std::string& name, bool* ok=0) {
    if(ok) *ok = true;
    if(     name == "none")       return COLOR_NONE;
    else if(name == "srgb")       return COLOR_SRGB;
    else if(name == "linear_rgb") return COLOR_LINEAR_RGB;
    else if(name == "cie_xyz")    return COLOR_CIE_XYZ;
    else if(name == "cie_lab")    return COLOR_CIE_LAB;

    if(ok) *ok = false;
    return COLOR_NONE;
}


inline const char* getColorSpaceName(ColorSpace cs) {
    switch(cs) {
    case COLOR_NONE:        return "none";
    case COLOR_SRGB:        return "srgb";
    case COLOR_LINEAR_RGB:  return "linear_rgb";
    case COLOR_CIE_XYZ:     return "cie_xyz";
    case COLOR_CIE_LAB:     return "cie_lab";
    }
    return "none";
}


inline Eigen::Vector3f convertColor(const Eigen::Vector3f& fromColor,
                                    ColorSpace from, ColorSpace to)
{
    if(from == COLOR_NONE || to == COLOR_NONE || from == to)
        return fromColor;

    Eigen::Vector3f color = fromColor;

    // To XYZ
    switch(from) {
    case COLOR_NONE:
        // Handled above.
        break;
    case COLOR_SRGB:
        color = linearRGBFromSrgb(color);
        if(to == COLOR_LINEAR_RGB) return color;
        // Fall-through
    case COLOR_LINEAR_RGB:
        color = cieXYZFromLinearRGB(color);
        break;
    case COLOR_CIE_XYZ:
        // Do nothing.
        break;
    case COLOR_CIE_LAB:
        color = cieXYZFromCieLab(color);
        break;
    }

    // From XYZ
    switch(to) {
    case COLOR_NONE:
        // Handled above.
        break;
    case COLOR_SRGB:
    case COLOR_LINEAR_RGB:
        color = linearRGBFromCieXYZ(color);
        if(to == COLOR_SRGB) {
            color = srgbFromLinearRGB(color);
        }
        break;
    case COLOR_CIE_XYZ:
        // Do nothing.
        break;
    case COLOR_CIE_LAB:
        color = cieLabFromCieXYZ(color);
        break;
    }

    return color;
}


}


#endif
