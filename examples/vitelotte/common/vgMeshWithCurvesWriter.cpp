#include "vgMeshWithCurvesWriter.h"


VGMeshWithCurveWriter::VGMeshWithCurveWriter(Version version)
    : Base(version) {
}


void VGMeshWithCurveWriter::write(std::ostream& _out, const Mesh& mesh) {
    typedef Mesh::Halfedge Halfedge;
    typedef Mesh::PointConstraint PointConstraint;
    typedef Mesh::Curve Curve;

    Base::write(_out, mesh);

    for(unsigned pci = 0; pci < mesh.nPointConstraints(); ++pci)
    {
        PointConstraint pc(pci);

        if(mesh.isValueConstraint(pc))
        {
            _out << "pvc " << vertexIndex(mesh.vertex(pc))
                 << " " << mesh.value(pc).transpose().format(m_format) << "\n";
        }
        if(mesh.isGradientConstraint(pc))
        {
            _out << "pgc " << vertexIndex(mesh.vertex(pc))
                 << " " << mesh.gradient(pc).transpose().format(m_format) << "\n";
        }
//        _out << "pc " << (mesh.isValueConstraint(pc)? "o": "x") << " "
//             << (mesh.isGradientConstraint(pc)? "o": "x") << ";";
//        if(mesh.isValueConstraint(pc))
//        {
//            _out << " " << mesh.value(pc).transpose().format(m_format) << ";";
//        }
//        if(mesh.isGradientConstraint(pc))
//        {
//            _out << " " << mesh.gradient(pc).transpose().format(m_format) << ";";
//        }
//        _out << " " << vertexIndex(mesh.vertex(pc)) << "\n";
    }

    for(unsigned ci = 0; ci < mesh.nCurves(); ++ci)
    {
        Curve curve(ci);

        Halfedge h = mesh.firstHalfedge(curve);
        _out << "c " << vertexIndex(mesh.fromVertex(h)) << " " << mesh.fromCurvePos(h);
        while(h.isValid())
        {
            _out << " " << vertexIndex(mesh.toVertex(h)) << " " << mesh.toCurvePos(h);
            h = mesh.nextCurveHalfedge(h);
        }
        _out << "\n";

        if(mesh.bezierCurve(curve).nPoints())
        {
            typedef Mesh::BezierCurve BezierCurve;
            const BezierCurve& bc = mesh.bezierCurve(curve);
            _out << "bp " << ci << " M " << bc.point(0).transpose().format(m_format);
            for(unsigned si = 0; si < bc.nSegments(); ++si)
            {
                switch(bc.type(si))
                {
                case BezierCurve::LINEAR:    _out << " L"; break;
                case BezierCurve::QUADRATIC: _out << " Q"; break;
                case BezierCurve::CUBIC:     _out << " C"; break;
                }
                for(unsigned pi = 1; pi < bc.nPoints(si); ++pi)
                {
                    _out << " " << bc.point(si, pi).transpose().format(m_format);
                }
            }
            _out << "\n";
        }

        if(mesh.valueTear(curve))    _out << "dcvTear " << ci << "\n";
        if(mesh.gradientTear(curve)) _out << "dcgTear " << ci << "\n";

        if(!mesh.valueFunction(curve, Mesh::VALUE_LEFT).empty())
        {
            _out << (mesh.valueTear(curve)? "dcvLeft ": "dcv ") << ci;
            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::VALUE_LEFT));
            _out << "\n";
        }
        if(mesh.valueTear(curve) && !mesh.valueFunction(curve, Mesh::VALUE_RIGHT).empty())
        {
            _out << "dcvRight " << ci;
            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::VALUE_RIGHT));
            _out << "\n";
        }

        if(!mesh.valueFunction(curve, Mesh::GRADIENT_LEFT).empty())
        {
            _out << (mesh.gradientTear(curve)? "dcgLeft ": "dcg ") << ci;
            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::GRADIENT_LEFT));
            _out << "\n";
        }
        if(mesh.gradientTear(curve) && !mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT).empty())
        {
            _out << "dcgRight " << ci;
            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT));
            _out << "\n";
        }
//        _out << "dc"
//             << " " << (mesh.valueFunction(curve, Mesh::VALUE_LEFT).empty()? "x": "o");
//        if(mesh.valueTear(curve))
//        {
//            _out << "/" << (mesh.valueFunction(curve, Mesh::VALUE_RIGHT).empty()? "x": "o");
//        }
//        _out << " " << (mesh.valueFunction(curve, Mesh::GRADIENT_LEFT).empty()? "x": "o");
//        if(mesh.gradientTear(curve))
//        {
//            _out << "/" << (mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT).empty()? "x": "o");
//        }
//        _out << ";";

//        if(!mesh.valueFunction(curve, Mesh::VALUE_LEFT).empty())
//        {
//            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::VALUE_LEFT));
//        }
//        if(mesh.valueTear(curve) && !mesh.valueFunction(curve, Mesh::VALUE_RIGHT).empty())
//        {
//            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::VALUE_RIGHT));
//        }

//        if(!mesh.valueFunction(curve, Mesh::GRADIENT_LEFT).empty())
//        {
//            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::GRADIENT_LEFT));
//        }
//        if(mesh.gradientTear(curve) && !mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT).empty())
//        {
//            writeValueFunction(_out, mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT));
//        }

//        Halfedge h = mesh.firstHalfedge(curve);
//        _out << " " << vertexIndex(mesh.fromVertex(h)) << ":" << mesh.fromCurvePos(h);
//        while(h.isValid())
//        {
//            _out << " " << vertexIndex(mesh.toVertex(h)) << ":" << mesh.toCurvePos(h);
//            h = mesh.nextCurveHalfedge(h);
//        }
//        _out << "\n";
    }
}


void VGMeshWithCurveWriter::writeValueFunction(std::ostream& out, const ValueFunction& vg) const
{
    for(ValueFunction::ConstIterator stop = vg.begin();
        stop != vg.end(); ++stop)
    {
        out << "   " << stop->first << " " << stop->second.transpose().format(m_format);
    }
}
