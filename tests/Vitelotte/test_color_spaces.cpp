#include <fstream>
#include <iostream>
#include <vector>

#include <Patate/common/gl_utils/color.h>


using namespace PatateCommon;

typedef Eigen::Vector3f Color;
typedef Eigen::Matrix<Color, Eigen::Dynamic, Eigen::Dynamic> Image;


struct ColorPair {
    Color from, to;
    inline ColorPair(const Color& from, const Color& to)
        : from(from), to(to) {}
};

int byteFromFloat(float f) {
    return std::max(0, std::min(255, int(f * 256)));
}


void saveImage(const Image& img, const std::string& filename) {
    std::ofstream out(filename.c_str());
    assert(out.good());

    out << "P3\n";
    out << img.rows() << " " << img.cols() << "\n";
    out << "255\n";

    for(unsigned y = 0; y < img.cols(); ++y) {
        for(unsigned x = 0; x < img.rows(); ++x) {
            if(x != 0) out << " ";
            out << byteFromFloat(img(x, y)(0)) << " "
                << byteFromFloat(img(x, y)(1)) << " "
                << byteFromFloat(img(x, y)(2));
        }
        out << "\n";
    }
}


void drawGradient(Image& img,
                  const Color& from, const Color& to,
                  Color (*conv)(const Color&),
                  unsigned gw, unsigned gh,
                  unsigned ow, unsigned oh) {
    for(unsigned y = 0; y < gh; ++y) {
        for(unsigned x = 0; x < gw; ++x) {
            float a = float(x) / (gw-1);
            img(x+ow, y+oh) = conv((1.-a)*from + a*to);
        }
    }
}


Color identity(const Color& c) {
    return c;
}

Color srgbFromXYZ(const Color& c) {
    return srgbFromLinearRGB(linearRGBFromCieXYZ(c));
}
Color srgbFromLab(const Color& c) {
    return srgbFromXYZ(cieXYZFromCieLab(c));
}

Color XYZFromSrgb(const Color& c) {
    return cieXYZFromLinearRGB(linearRGBFromSrgb(c));
}

Color labFromSrgb(const Color& c) {
    return cieLabFromCieXYZ(XYZFromSrgb(c));
}


int main(int argc, char** argv) {
    std::string outFilename = "./test.ppm";

    typedef std::vector<ColorPair> ColorVector;
    typedef ColorVector::const_iterator ColorVectorIt;

    ColorVector colors;
    colors.push_back(ColorPair(Color(0, 0, 0), Color(1, 1, 1)));
    colors.push_back(ColorPair(Color(1, 0, 0), Color(0, 1, 0)));
    colors.push_back(ColorPair(Color(1, 0, 0), Color(1, 1, 0)));
    colors.push_back(ColorPair(Color(1, 0, 0), Color(1, 1, 1)));

    unsigned gw = 256;
    unsigned gh = 64;

    Color from(1, 0, 0);
    Color to(0, 1, 0);
//    Color from(0, 0, 0);
//    Color to(1, 1, 1);

    unsigned w = gw;
    unsigned h = 3 * gh * colors.size();

    Image img(w, h);

    unsigned row = 0;
    for(ColorVectorIt it = colors.begin();
        it != colors.end(); ++it) {
        drawGradient(img,
                     it->from,
                     it->to,
                     identity,
                     gw, gh, 0, (row++)*gh);
        drawGradient(img,
                     linearRGBFromSrgb(it->from),
                     linearRGBFromSrgb(it->to),
                     srgbFromLinearRGB,
                     gw, gh, 0, (row++)*gh);
        drawGradient(img,
                     labFromSrgb(it->from),
                     labFromSrgb(it->to),
                     srgbFromLab,
                     gw, gh, 0, (row++)*gh);
    }

//    Color c(.01, .01, .01);
//    std::cout << "c srgb: " << c.transpose() << "\n";
//    c = linearRGBFromSrgb(c);
//    std::cout << "c lrgb: " << c.transpose() << "\n";
//    c = cieXYZFromLinearRGB(c);
//    std::cout << "c xyz : " << c.transpose() << "\n";
//    c = cieLabFromCieXYZ(c);
//    std::cout << "c lab : " << c.transpose() << "\n";
//    c = cieXYZFromCieLab(c);
//    std::cout << "c xyz : " << c.transpose() << "\n";
//    c = linearRGBFromCieXYZ(c);
//    std::cout << "c lrgb: " << c.transpose() << "\n";
//    c = srgbFromLinearRGB(c);
//    std::cout << "c srgb: " << c.transpose() << "\n";

//    typedef internal::LabHelper<float> LH;
//    float test = 0.0001;
//    std::cout << "test: " << test << "\n";
//    test = LH::f(test);
//    std::cout << "test: " << test << "\n";
//    test = LH::fInv(test);
//    std::cout << "test: " << test << "\n";

    saveImage(img, outFilename);

    return 0;
}
