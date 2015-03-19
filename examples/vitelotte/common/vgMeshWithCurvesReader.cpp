#include <string>
#include <sstream>

#include "vgMeshWithCurvesReader.h"


VGMeshWithCurveReader::VGMeshWithCurveReader()
{

}


bool VGMeshWithCurveReader::parseDefinition(const std::string& spec,
                                            std::istream& def)
{
    if(spec == "pc")
    {
        parsePointConstraint(def);
        return true;
    }
    else if(spec == "dc")
    {
        parseDc(def);
        return true;
    }

    return Base::parseDefinition(spec, def);
}

void VGMeshWithCurveReader::parsePointConstraint(std::istream& def)
{
#define VITELOTTE_READER_ERROR(_msg) do { error(_msg); return; } while(0)
    if(!def.good())
        VITELOTTE_READER_ERROR("empty point constraint declaration");
    std::getline(def, m_part, ';');
    m_in.str(m_part);
    m_in.seekg(std::ios_base::beg);

    char cd, cg;
    m_in >> cd >> cg >> std::ws;
    bool diffuse = (cd == 'o');
    bool gradient = (cg == 'o');

    if((!diffuse && cd != 'x') || (!gradient && cg != 'x') || m_in.fail() || !m_in.eof())
        VITELOTTE_READER_ERROR("invalid point constraint type specification.");

    Mesh::PointConstraint pc = m_mesh->addPointConstraint();

    if(diffuse)
    {
        if(!def.good())
            VITELOTTE_READER_ERROR("expected point constraint value specification.");
        std::getline(def, m_part, ';');
        m_in.str(m_part);
        m_in.seekg(std::ios_base::beg);

        Mesh::Value& nv = m_mesh->value(pc);
        for(unsigned i = 0; i < m_mesh->nCoeffs(); ++i)
            m_in >> nv(i);

        m_in >> std::ws;
        if(m_in.fail() || !m_in.eof())
            VITELOTTE_READER_ERROR("invalid point constraint value specification.");
    }
    if(gradient)
    {
        if(!def.good())
            VITELOTTE_READER_ERROR("expected point constraint gradient specification.");
        std::getline(def, m_part, ';');
        m_in.str(m_part);
        m_in.seekg(std::ios_base::beg);

        Mesh::Gradient& ng = m_mesh->gradient(pc);
        for(unsigned j = 0; j < m_mesh->nDims(); ++j)
            for(unsigned i = 0; i < m_mesh->nCoeffs(); ++i)
                m_in >> ng(i, j);

        m_in >> std::ws;
        if(m_in.fail() || !m_in.eof())
            VITELOTTE_READER_ERROR("invalid point constraint gradient specification.");
    }

    if(!def.good())
        VITELOTTE_READER_ERROR("expected point constraint vertex specification.");

    int vi;
    def >> vi >> std::ws;
    m_mesh->vertex(pc) = Mesh::Vertex(vi);
    if(!m_mesh->isValid(m_mesh->vertex(pc)))
        VITELOTTE_READER_ERROR("specified vertex in invalid.");

    if(def.fail() || !def.eof())
        VITELOTTE_READER_ERROR("invalid point constraint vertex specification.");

#undef VITELOTTE_READER_ERROR
}


void VGMeshWithCurveReader::parseDc(std::istream& def)
{
    if(def.fail())
    {
        error("Incomplete diffusion curve definition. Expected type specification.");
        return;
    }
    std::getline(def, m_part, ';');
    m_in.str(m_part);
    m_in.seekg(std::ios_base::beg);

    int valueType = parseCurveType(m_in);
    int gradientType = parseCurveType(m_in);
    if(valueType < 0 || gradientType < 0)
        return;

    Mesh::Curve c = m_mesh->addCurve(
            ((valueType    & TEAR)? Mesh::VALUE_TEAR:    0) |
            ((gradientType & TEAR)? Mesh::GRADIENT_TEAR: 0));

    Mesh::ValueFunction g;
    if(valueType & CONS_LEFT) {
        if(!parseGradient(def, g))
            return;
        m_mesh->valueFunction(c, Mesh::VALUE_LEFT) = g;
    }
    if(valueType & CONS_RIGHT) {
        if(!parseGradient(def, g))
            return;
        m_mesh->valueFunction(c, Mesh::VALUE_RIGHT) = g;
    }
    if(gradientType & CONS_LEFT) {
        if(!parseGradient(def, g))
            return;
        m_mesh->valueFunction(c, Mesh::GRADIENT_LEFT) = g;
    }
    if(gradientType & CONS_RIGHT) {
        if(!parseGradient(def, g))
            return;
        m_mesh->valueFunction(c, Mesh::GRADIENT_RIGHT) = g;
    }

    if(!parseCurveVertices(def, c))
        return;

    def >> std::ws;
    if(!def.eof())
        error("Unexpected part, expected new line.");
}


int VGMeshWithCurveReader::parseCurveType(std::istream& in)
{
    unsigned type = 0;
    in >> std::ws >> m_token;

    if(m_token.size() == 3)
    {
        type = TEAR;
        if(m_token[2] == 'o')
            type |= CONS_RIGHT;
        else if(m_token[2] != 'x')
            goto fail;
    }
    else if(m_token.size() != 1)
        goto fail;

    if(m_token[0] == 'o')
        type |= CONS_LEFT;
    else if(m_token[0] != 'x')
        goto fail;

    return type;

fail:
    error("Invalid diffusion curve type specifications.");
    return -1;
}

bool VGMeshWithCurveReader::parseGradient(std::istream& def, VGMeshWithCurves::ValueFunction &g)
{
    if(def.fail())
    {
        error("Incomplete diffusion curve definition. Expected gradient definition.");
        return false;
    }

    int nparam = m_mesh->nCoeffs();

    std::getline(def, m_part, ';');
    std::string::iterator c = m_part.begin();
    std::string::iterator cend = m_part.end();
    while(c != cend && std::isspace(*c)) ++c;
    while(c != cend)
    {
        for(int i = 0; i < nparam+1; ++i)
        {
            while(c != cend && (std::isdigit(*c) || *c == '.' || *c == '-')) ++c;

            bool ok = (i == 0)?      c != cend && *c == ':':
                      (i == nparam)? c == cend || std::isspace(*c):
                                     c != cend && *c == ',';
            if(!ok)
            {
                error("Invalid gradent specification.");
                return false;
            }

            if(c != cend)
                *c = ' ';
            while(c != cend && std::isspace(*c)) ++c;
        }
    }

    m_in.str(m_part);
    m_in.seekg(std::ios_base::beg);
    m_in >> std::ws;

    g.clear();
    Mesh::Value v(m_mesh->nCoeffs());
    while(m_in.good())
    {
        float pos;
        m_in >> pos;
        for(int i = 0; i < nparam; ++i)
            m_in >> v(i);
        g.add(pos, v);
    }

    if(m_in.fail())
    {
        error("Error while reading gradent specification.");
        return false;
    }

    return true;
}


bool VGMeshWithCurveReader::parseCurveVertices(std::istream& def, Mesh::Curve curve)
{
    if(def.fail())
    {
        error("Incomplete diffusion curve definition. Expected vertices specifications.");
        return false;
    }

    std::getline(def, m_part, ';');
    std::string::iterator c = m_part.begin();
    std::string::iterator cend = m_part.end();
    while(c != cend && std::isspace(*c)) ++c;
    while(c != cend)
    {
        while(c != cend && std::isdigit(*c)) ++c;
        if(c == cend || *c != ':')
            goto fail;
        *c = ' ';
        while(c != cend && std::isspace(*c)) ++c;

        while(c != cend && (std::isdigit(*c) || *c == '.')) ++c;
        if(c != cend && !std::isspace(*c))
            goto fail;
        while(c != cend && std::isspace(*c)) ++c;
    }

    m_in.str(m_part);
    m_in.seekg(std::ios_base::beg);

    int fromV;
    float fromP;
    m_in >> fromV >> fromP >> std::ws;
    while(m_in.good())
    {
        int toV;
        float toP;
        m_in >> toV >> toP >> std::ws;

        Mesh::Halfedge h = m_mesh->findHalfedge(Mesh::Vertex(fromV), Mesh::Vertex(toV));
        if(!h.isValid())
        {
            error("Invalid edge in diffusion curve specification.");
            return false;
        }
        m_mesh->addHalfedgeToCurve(curve, h, fromP, toP);

        fromV = toV;
        fromP = toP;
    }

    if(m_in.fail())
        goto fail;

    return true;

fail:
    error("Invalid vertices specification.");
    return false;
}
