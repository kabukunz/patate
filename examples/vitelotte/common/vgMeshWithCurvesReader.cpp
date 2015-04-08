#include <string>
#include <sstream>

#include "vgMeshWithCurvesReader.h"


#define PTT_ERROR_IF(_cond, _msg) do { if(_cond) { error(_msg); return true; } } while(false)
#define PTT_RETURN_IF_ERROR() do { if(m_error) { return true; } } while(false)


VGMeshWithCurveReader::VGMeshWithCurveReader()
{

}


bool VGMeshWithCurveReader::parseDefinition(const std::string& spec,
                                            std::istream& def, Mesh& mesh)
{
    // Ponctual Value Constraint
    if(spec == "pvc")
    {
        unsigned vi;
        def >> vi;
        PTT_ERROR_IF(!def || vi >= mesh.nVertices(), "Invalid vertex index");
        parseValue(def); PTT_RETURN_IF_ERROR();
        if(!def.eof()) warning("Too much component in point value constraint declaration");

        PointConstraint pc = mesh.pointConstraint(Vertex(vi));
        if(!pc.isValid()) pc = mesh.addPointConstraint(Vertex(vi));
        mesh.value(pc) = m_value;
    }
    // Ponctual Gradient Constraint
    else if(spec == "pgc")
    {
        unsigned vi;
        def >> vi;
        PTT_ERROR_IF(!def || vi >= mesh.nVertices(), "Invalid vertex index");
        parseGradient(def); PTT_RETURN_IF_ERROR();
        if(!def.eof()) warning("Too much component in point gradient constraint declaration");

        PointConstraint pc = mesh.pointConstraint(Vertex(vi));
        if(!pc.isValid()) pc = mesh.addPointConstraint(Vertex(vi));
        mesh.gradient(pc) = m_gradient;

        return true;
    }
    // Curve
    else if(spec == "c")
    {
        unsigned vi, prevVi;
        float    pos, prevPos;
        Curve c = mesh.addCurve(0);
        def >> prevVi;  PTT_ERROR_IF(!def || prevVi >= mesh.nVertices(), "Invalid vertex index");
        def >> prevPos; PTT_ERROR_IF(!def, "Invalid vertex position");

        while(def && !def.eof())
        {
            def >> vi;  PTT_ERROR_IF(!def || vi >= mesh.nVertices(), "Invalid vertex index");
            def >> pos; PTT_ERROR_IF(!def, "Invalid vertex position");
            PTT_ERROR_IF(!def, "Invalid curve declaration");
            Halfedge h = mesh.findHalfedge(Vertex(prevVi), Vertex(vi));
            PTT_ERROR_IF(!h.isValid(), "Invalid curve: prescribed edge does not exist");
            mesh.addHalfedgeToCurve(c, h, prevPos, pos);
            prevVi  = vi;
            prevPos = pos;
        }
        return true;
    }
    // Diffusion Curve Value Tear / Diffusion Curve Gradient Tear
    else if(spec == "dcvTear" || spec == "dcgTear")
    {
        unsigned ci;
        def >> ci; PTT_ERROR_IF(!def || ci >= mesh.nCurves(), "Invalid curve index");
        unsigned flag = (spec == "dcvTear")? Mesh::VALUE_TEAR: Mesh::GRADIENT_TEAR;
        mesh.setFlags(Curve(ci), mesh.flags(Curve(ci)) | flag);
    }
    // Diffusion Curve Value (Left/Right) / Diffusion Curve Gradient (Left/Right)
    else if(spec == "dcv" || spec == "dcvLeft" || spec == "dcvRight"
            || spec == "dcg" || spec == "dcgLeft" || spec == "dcgRight")
    {
        unsigned ci;
        def >> ci; PTT_ERROR_IF(!def || ci >= mesh.nCurves(), "Invalid curve index");

        // Update flags if required -- must be done before functionValue call
        unsigned flags = mesh.flags(Curve(ci));
        if(spec == "dcvLeft" || spec == "dcvRight") flags |= Mesh::VALUE_TEAR;
        if(spec == "dcgLeft" || spec == "dcgRight") flags |= Mesh::GRADIENT_TEAR;
        mesh.setFlags(Curve(ci), flags);

        unsigned which =
                (spec == "dcv" || spec == "dcvLeft" )? Mesh::VALUE_LEFT:
                (                 spec == "dcvRight")? Mesh::VALUE_RIGHT:
                (spec == "dcg" || spec == "dcgLeft" )? Mesh::GRADIENT_LEFT:
                                                       Mesh::GRADIENT_RIGHT;
        ValueFunction& vf = mesh.valueFunction(Curve(ci), which);
        while(def && !def.eof())
        {
            float pos;
            def >> pos; PTT_ERROR_IF(!def, "Invalid parameter");
            parseValue(def); PTT_RETURN_IF_ERROR();
            vf.add(pos, m_value);
        }
    }
    // Bezier Path
    else if(spec == "bp")
    {
        typedef typename Mesh::BezierCurve BezierCurve;
        typedef typename BezierCurve::SegmentType SegmentType;

        unsigned ci;
        def >> ci; PTT_ERROR_IF(!def || ci >= mesh.nCurves(), "Invalid curve index");

        BezierCurve& bc = mesh.bezierCurve(Curve(ci));
        if(bc.nPoints() != 0)
        {
            warning("Bezier curve already defined");
            return true;
        }

        def >> m_part; PTT_ERROR_IF(!def || m_part != "M", "Expected M path command");
        parseVector(def); PTT_RETURN_IF_ERROR();
        bc.setFirstPoint(m_vector);

        SegmentType type;
        unsigned count = 0;
        Vector points[3];
        for(unsigned i = 0; i < 3; ++i) points[i].resize(mesh.nDims());
        def >> std::ws;
        while(def && !def.eof())
        {
            if(std::isalpha(def.peek()))
            {
                PTT_ERROR_IF(count != 0, "unexpected path command");
                def >> m_part; PTT_ERROR_IF(!def || m_part.size() != 1, "Invalid string in path definition");
                switch(m_part[0])
                {
                case 'L': type = BezierCurve::LINEAR;    break;
                case 'Q': type = BezierCurve::QUADRATIC; break;
                case 'C': type = BezierCurve::CUBIC;     break;
                }
            }
            else
            {
                parseVector(def); PTT_RETURN_IF_ERROR();
                points[count] = m_vector;
                ++count;

                if(count == BezierCurve::size(type) - 1)
                {
                    bc.addSegment(type, points);
                    count = 0;
                }
            }
            def >> std::ws;
        }
    }
    else
    {
        return Base::parseDefinition(spec, def, mesh);
    }
    return true;
}

#undef PTT_ERROR_IF
