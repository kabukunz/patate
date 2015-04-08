#include <string>
#include <sstream>

#include "vgMeshWithCurvesReader.h"


#define PTT_ERROR_IF(_cond, _msg) do { if(_cond) { error(_msg); return true; } } while(false)
#define PTT_RETURN_IF_ERROR() do { if(m_error) { return true; } } while(false)


VGMeshWithCurveReader::VGMeshWithCurveReader()
{

}


bool VGMeshWithCurveReader::parseDefinition(const std::string& spec,
                                            std::istream& def)
{
    // Ponctual Value Constraint
    if(spec == "pvc")
    {
        unsigned vi;
        def >> vi;
        PTT_ERROR_IF(!def || vi >= m_mesh->nVertices(), "Invalid vertex index");
        parseValue(def); PTT_RETURN_IF_ERROR();
        if(!def.eof()) warning("Too much component in point value constraint declaration");

        PointConstraint pc = m_mesh->pointConstraint(Vertex(vi));
        if(!pc.isValid()) pc = m_mesh->addPointConstraint(Vertex(vi));
        m_mesh->value(pc) = m_value;
    }
    // Ponctual Gradient Constraint
    else if(spec == "pgc")
    {
        unsigned vi;
        def >> vi;
        PTT_ERROR_IF(!def || vi >= m_mesh->nVertices(), "Invalid vertex index");
        parseGradient(def); PTT_RETURN_IF_ERROR();
        if(!def.eof()) warning("Too much component in point gradient constraint declaration");

        PointConstraint pc = m_mesh->pointConstraint(Vertex(vi));
        if(!pc.isValid()) pc = m_mesh->addPointConstraint(Vertex(vi));
        m_mesh->gradient(pc) = m_gradient;

        return true;
    }
    // Curve
    else if(spec == "c")
    {
        unsigned vi, prevVi;
        float    pos, prevPos;
        Curve c = m_mesh->addCurve(0);
        def >> prevVi;  PTT_ERROR_IF(!def || prevVi >= m_mesh->nVertices(), "Invalid vertex index");
        def >> prevPos; PTT_ERROR_IF(!def, "Invalid vertex position");

        while(def && !def.eof())
        {
            def >> vi;  PTT_ERROR_IF(!def || vi >= m_mesh->nVertices(), "Invalid vertex index");
            def >> pos; PTT_ERROR_IF(!def, "Invalid vertex position");
            PTT_ERROR_IF(!def, "Invalid curve declaration");
            Halfedge h = m_mesh->findHalfedge(Vertex(prevVi), Vertex(vi));
            PTT_ERROR_IF(!h.isValid(), "Invalid curve: prescribed edge does not exist");
            m_mesh->addHalfedgeToCurve(c, h, prevPos, pos);
            prevVi  = vi;
            prevPos = pos;
        }
        return true;
    }
    // Diffusion Curve Value Tear / Diffusion Curve Gradient Tear
    else if(spec == "dcvTear" || spec == "dcgTear")
    {
        unsigned ci;
        def >> ci; PTT_ERROR_IF(!def || ci >= m_mesh->nCurves(), "Invalid curve index");
        unsigned flag = (spec == "dcvTear")? Mesh::VALUE_TEAR: Mesh::GRADIENT_TEAR;
        m_mesh->setFlags(Curve(ci), m_mesh->flags(Curve(ci)) | flag);
    }
    // Diffusion Curve Value (Left/Right) / Diffusion Curve Gradient (Left/Right)
    else if(spec == "dcv" || spec == "dcvLeft" || spec == "dcvRight"
            || spec == "dcg" || spec == "dcgLeft" || spec == "dcgRight")
    {
        unsigned ci;
        def >> ci; PTT_ERROR_IF(!def || ci >= m_mesh->nCurves(), "Invalid curve index");

        // Update flags if required -- must be done before functionValue call
        unsigned flags = m_mesh->flags(Curve(ci));
        if(spec == "dcvLeft" || spec == "dcvRight") flags |= Mesh::VALUE_TEAR;
        if(spec == "dcgLeft" || spec == "dcgRight") flags |= Mesh::GRADIENT_TEAR;
        m_mesh->setFlags(Curve(ci), flags);

        unsigned which =
                (spec == "dcv" || spec == "dcvLeft" )? Mesh::VALUE_LEFT:
                (                 spec == "dcvRight")? Mesh::VALUE_RIGHT:
                (spec == "dcg" || spec == "dcgLeft" )? Mesh::GRADIENT_LEFT:
                                                       Mesh::GRADIENT_RIGHT;
        ValueFunction& vf = m_mesh->valueFunction(Curve(ci), which);
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
        def >> ci; PTT_ERROR_IF(!def || ci >= m_mesh->nCurves(), "Invalid curve index");

        BezierCurve& bc = m_mesh->bezierCurve(Curve(ci));
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
        for(unsigned i = 0; i < 3; ++i) points[i].resize(m_mesh->nDims());
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
        return Base::parseDefinition(spec, def);
    }
    return true;
}

//void VGMeshWithCurveReader::parsePointConstraint(std::istream& def)
//{
//#define VITELOTTE_READER_ERROR(_msg) do { error(_msg); return; } while(0)
//    if(!def.good())
//        VITELOTTE_READER_ERROR("empty point constraint declaration");
//    std::getline(def, m_part, ';');
//    m_in.str(m_part);
//    m_in.seekg(std::ios_base::beg);

//    char cd, cg;
//    m_in >> cd >> cg >> std::ws;
//    bool diffuse = (cd == 'o');
//    bool gradient = (cg == 'o');

//    if((!diffuse && cd != 'x') || (!gradient && cg != 'x') || m_in.fail() || !m_in.eof())
//        VITELOTTE_READER_ERROR("invalid point constraint type specification.");

//    Mesh::PointConstraint pc = m_mesh->addPointConstraint();

//    if(diffuse)
//    {
//        if(!def.good())
//            VITELOTTE_READER_ERROR("expected point constraint value specification.");
//        std::getline(def, m_part, ';');
//        m_in.str(m_part);
//        m_in.seekg(std::ios_base::beg);

//        Mesh::Value& nv = m_mesh->value(pc);
//        for(unsigned i = 0; i < m_mesh->nCoeffs(); ++i)
//            m_in >> nv(i);

//        m_in >> std::ws;
//        if(m_in.fail() || !m_in.eof())
//            VITELOTTE_READER_ERROR("invalid point constraint value specification.");
//    }
//    if(gradient)
//    {
//        if(!def.good())
//            VITELOTTE_READER_ERROR("expected point constraint gradient specification.");
//        std::getline(def, m_part, ';');
//        m_in.str(m_part);
//        m_in.seekg(std::ios_base::beg);

//        Mesh::Gradient& ng = m_mesh->gradient(pc);
//        for(unsigned j = 0; j < m_mesh->nDims(); ++j)
//            for(unsigned i = 0; i < m_mesh->nCoeffs(); ++i)
//                m_in >> ng(i, j);

//        m_in >> std::ws;
//        if(m_in.fail() || !m_in.eof())
//            VITELOTTE_READER_ERROR("invalid point constraint gradient specification.");
//    }

//    if(!def.good())
//        VITELOTTE_READER_ERROR("expected point constraint vertex specification.");

//    int vi;
//    def >> vi >> std::ws;
//    m_mesh->vertex(pc) = Mesh::Vertex(vi);
//    if(!m_mesh->isValid(m_mesh->vertex(pc)))
//        VITELOTTE_READER_ERROR("specified vertex in invalid.");

//    if(def.fail() || !def.eof())
//        VITELOTTE_READER_ERROR("invalid point constraint vertex specification.");

//#undef VITELOTTE_READER_ERROR
//}


//void VGMeshWithCurveReader::parseDc(std::istream& def)
//{
//    if(def.fail())
//    {
//        error("Incomplete diffusion curve definition. Expected type specification.");
//        return;
//    }
//    std::getline(def, m_part, ';');
//    m_in.str(m_part);
//    m_in.seekg(std::ios_base::beg);

//    int valueType = parseCurveType(m_in);
//    int gradientType = parseCurveType(m_in);
//    if(valueType < 0 || gradientType < 0)
//        return;

//    Mesh::Curve c = m_mesh->addCurve(
//            ((valueType    & TEAR)? Mesh::VALUE_TEAR:    0) |
//            ((gradientType & TEAR)? Mesh::GRADIENT_TEAR: 0));

//    Mesh::ValueFunction g;
//    if(valueType & CONS_LEFT) {
//        if(!parseGradient(def, g))
//            return;
//        m_mesh->valueFunction(c, Mesh::VALUE_LEFT) = g;
//    }
//    if(valueType & CONS_RIGHT) {
//        if(!parseGradient(def, g))
//            return;
//        m_mesh->valueFunction(c, Mesh::VALUE_RIGHT) = g;
//    }
//    if(gradientType & CONS_LEFT) {
//        if(!parseGradient(def, g))
//            return;
//        m_mesh->valueFunction(c, Mesh::GRADIENT_LEFT) = g;
//    }
//    if(gradientType & CONS_RIGHT) {
//        if(!parseGradient(def, g))
//            return;
//        m_mesh->valueFunction(c, Mesh::GRADIENT_RIGHT) = g;
//    }

//    if(!parseCurveVertices(def, c))
//        return;

//    def >> std::ws;
//    if(!def.eof())
//        error("Unexpected part, expected new line.");
//}


//int VGMeshWithCurveReader::parseCurveType(std::istream& in)
//{
//    unsigned type = 0;
//    in >> std::ws >> m_token;

//    if(m_token.size() == 3)
//    {
//        type = TEAR;
//        if(m_token[2] == 'o')
//            type |= CONS_RIGHT;
//        else if(m_token[2] != 'x')
//            goto fail;
//    }
//    else if(m_token.size() != 1)
//        goto fail;

//    if(m_token[0] == 'o')
//        type |= CONS_LEFT;
//    else if(m_token[0] != 'x')
//        goto fail;

//    return type;

//fail:
//    error("Invalid diffusion curve type specifications.");
//    return -1;
//}

//bool VGMeshWithCurveReader::parseGradient(std::istream& def, VGMeshWithCurves::ValueFunction &g)
//{
//    if(def.fail())
//    {
//        error("Incomplete diffusion curve definition. Expected plf definition.");
//        return false;
//    }

//    int nparam = m_mesh->nCoeffs();

//    std::getline(def, m_part, ';');
//    std::string::iterator c = m_part.begin();
//    std::string::iterator cend = m_part.end();
//    while(c != cend && std::isspace(*c)) ++c;
//    while(c != cend)
//    {
//        for(int i = 0; i < nparam+1; ++i)
//        {
//            while(c != cend && (std::isdigit(*c) || *c == '.' || *c == '-')) ++c;

//            bool ok = (i == 0)?      c != cend && *c == ':':
//                      (i == nparam)? c == cend || std::isspace(*c):
//                                     c != cend && *c == ',';
//            if(!ok)
//            {
//                error("Invalid plf specification.");
//                return false;
//            }

//            if(c != cend)
//                *c = ' ';
//            while(c != cend && std::isspace(*c)) ++c;
//        }
//    }

//    m_in.str(m_part);
//    m_in.seekg(std::ios_base::beg);
//    m_in >> std::ws;

//    g.clear();
//    Mesh::Value v(m_mesh->nCoeffs());
//    while(m_in.good())
//    {
//        float pos;
//        m_in >> pos;
//        for(int i = 0; i < nparam; ++i)
//            m_in >> v(i);
//        g.add(pos, v);
//    }

//    if(m_in.fail())
//    {
//        error("Error while reading plf specification.");
//        return false;
//    }

//    return true;
//}


//bool VGMeshWithCurveReader::parseCurveVertices(std::istream& def, Mesh::Curve curve)
//{
//    if(def.fail())
//    {
//        error("Incomplete diffusion curve definition. Expected vertices specifications.");
//        return false;
//    }

//    std::getline(def, m_part, ';');
//    std::string::iterator c = m_part.begin();
//    std::string::iterator cend = m_part.end();
//    while(c != cend && std::isspace(*c)) ++c;
//    while(c != cend)
//    {
//        while(c != cend && std::isdigit(*c)) ++c;
//        if(c == cend || *c != ':')
//            goto fail;
//        *c = ' ';
//        while(c != cend && std::isspace(*c)) ++c;

//        while(c != cend && (std::isdigit(*c) || *c == '.')) ++c;
//        if(c != cend && !std::isspace(*c))
//            goto fail;
//        while(c != cend && std::isspace(*c)) ++c;
//    }

//    m_in.str(m_part);
//    m_in.seekg(std::ios_base::beg);

//    int fromV;
//    float fromP;
//    m_in >> fromV >> fromP >> std::ws;
//    while(m_in.good())
//    {
//        int toV;
//        float toP;
//        m_in >> toV >> toP >> std::ws;

//        Mesh::Halfedge h = m_mesh->findHalfedge(Mesh::Vertex(fromV), Mesh::Vertex(toV));
//        if(!h.isValid())
//        {
//            error("Invalid edge in diffusion curve specification.");
//            return false;
//        }
//        m_mesh->addHalfedgeToCurve(curve, h, fromP, toP);

//        fromV = toV;
//        fromP = toP;
//    }

//    if(m_in.fail())
//        goto fail;

//    return true;

//fail:
//    error("Invalid vertices specification.");
//    return false;
//}

#undef PTT_ERROR_IF
