/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <vector>
#include <iomanip>

#include "convertColorSpaceCommand.h"


void convertValueFunction(Mesh::ValueFunction& vf,
                          Vitelotte::ColorSpace from, Vitelotte::ColorSpace to) {
    for(Mesh::ValueFunction::Iterator stop = vf.begin();
        stop != vf.end(); ++stop) {

        Eigen::Vector3f c = stop->second.head<3>().cast<float>();
        c = convertColor(c, from, to);
        stop->second.head<3>() = c.cast<Mesh::Scalar>();
    }
}


bool hasNonZeroValue(Mesh::ValueFunction& vf) {
    for(Mesh::ValueFunction::Iterator stop = vf.begin();
        stop != vf.end(); ++stop) {
        if(stop->second != Mesh::Value::Zero(stop->second.rows())) {
            return true;
        }
    }
    return false;
}


ConvertColorSpaceCommand::ConvertColorSpaceCommand()
    : m_colorSpace(PatateCommon::COLOR_NONE)
{
}


bool ConvertColorSpaceCommand::parseArgs(int argc, char** argv, int& argi)
{
    if(argi < argc)
    {
        bool ok = false;
        m_colorSpace = PatateCommon::colorSpaceFromName(argv[argi++], &ok);
        if(!ok) {
            std::cerr << "Invalid color space specification.\n";
            return false;
        }
        return true;
    }
    return false;
}


bool ConvertColorSpaceCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Convert color space to "
                                        << PatateCommon::getColorSpaceName(m_colorSpace) << "...\n";
    if(mesh.nCoeffs() < 3 || mesh.nCoeffs() > 4) {
        std::cerr << "Can not apply color conversion on a mesh with a number of coefficients "
                     "not equal to 3 or 4 (3 channels + alpha).\n";
        return false;
    }

    bool warnInvalidGradient = mesh.hasEdgeGradient();
    for(unsigned ci = 0; ci < mesh.nCurves(); ++ci) {
        Mesh::Curve c(ci);

        convertValueFunction(mesh.valueFunction(c, Mesh::VALUE_LEFT),
                             mesh.colorSpace(), m_colorSpace);
        if(mesh.valueTear(c)) {
            convertValueFunction(mesh.valueFunction(c, Mesh::VALUE_RIGHT),
                                 mesh.colorSpace(), m_colorSpace);
        }

        if(hasNonZeroValue(mesh.valueFunction(c, Mesh::GRADIENT_LEFT))
           || (   mesh.gradientTear(c)
               && hasNonZeroValue(mesh.valueFunction(c, Mesh::GRADIENT_RIGHT)))) {
           std::cerr << "Warning: non-null gradient curve constraint. "
                        "Color space conversion may lead to unexpected results.\n";
           warnInvalidGradient = false;
        }
    }

    std::vector<bool> nmask(mesh.nodesSize(), true);
    for(Mesh::HalfedgeIterator hit = mesh.halfedgesBegin();
        hit != mesh.halfedgesEnd(); ++hit) {
        // For all _value_ nodes
        for(int i = 0; i < Mesh::EDGE_GRADIENT; ++i) {
            if(!mesh.hasAttribute(Mesh::HalfedgeAttribute(i))) continue;

            Mesh::Node n = mesh.halfedgeNode(*hit, Mesh::HalfedgeAttribute(i));
            if(!n.isValid() || !nmask[n.idx()] || !mesh.isConstraint(n)) continue;

            Eigen::Vector3f c = mesh.value(n).head<3>().cast<float>();
            c = convertColor(c, mesh.colorSpace(), m_colorSpace);
            mesh.value(n).head<3>() = c.cast<Mesh::Scalar>();

            nmask[n.idx()] = false;
        }
        if(warnInvalidGradient) {
            Mesh::Node n = mesh.edgeGradientNode(*hit);
            if(n.isValid() && mesh.isConstraint(n)
            && mesh.value(n) != Mesh::Value::Zero(mesh.nCoeffs())) {
                std::cerr << "Warning: non-null gradient node. "
                             "Color space conversion may lead to unexpected results.\n";
                warnInvalidGradient = false;
            }
        }

    }
    mesh.setColorSpace(m_colorSpace);

    return true;
}


const char* ConvertColorSpaceCommand::cmdOptions()
{
    return "COLOR_SPACE";
}


const char* ConvertColorSpaceCommand::cmdDesc()
{
    return "Convert the values from the mesh's declared color space to "
           "COLOR_SPACE. Accepted values are none, srgb, linear_srgb, cie_xyz "
           "and cie_lab. Warning: Do not work if there are gradient attributes.";
}
