
#ifndef _MVGWRITER_H_
#define _MVGWRITER_H_


#include <cassert>
#include <ostream>
#include <fstream>


namespace Vitelotte
{


/**
 * \brief The MVGWriter class allow to save FemMesh objects in `.mvg` files.
 *
 * \see FemMesh MVGReader
 */
template < typename _Mesh >
class MVGWriter
{
public:

    typedef _Mesh Mesh;
    typedef typename Mesh::Node Node;

    enum Version
    {
        Version1_0 = 0x100,
        LastestVersion = Version1_0
    };


public:

    /**
     * \brief Constructor
     */
    inline MVGWriter(Version version=LastestVersion)
        : m_version(version) {}

    /**
     * \brief Write `mesh` in the stream `out` as a `.mvg` file.
     * \param mesh The QMesh to save.
     * \param out The output stream.
     */
    void write(std::ostream& _out, const Mesh& mesh) const;


protected:
    void printNode(std::ostream& _out, Node n) const;


protected:
    Version m_version;
};


template < typename Mesh >
void writeMvg(std::ostream& out, const Mesh& mesh,
              typename MVGWriter<Mesh>::Version version=MVGWriter<Mesh>::LastestVersion);

template < typename Mesh >
void writeMvgToFile(const std::string& filename, const Mesh& mesh,
                    typename MVGWriter<Mesh>::Version version=MVGWriter<Mesh>::LastestVersion);


} // namespace Vitelotte

#include "mvgWriter.hpp"


#endif // _QVGWRITER_H_
