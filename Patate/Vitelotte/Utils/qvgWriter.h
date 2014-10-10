
#ifndef _QVGWRITER_H_
#define _QVGWRITER_H_


#include <cassert>
#include <ostream>


namespace Vitelotte
{


/**
 * \brief The QVGWriter class allow to save QMesh objects in `.qvg` files.
 *
 * \see QMesh QVGReader
 */
template < typename _Mesh >
class QVGWriter
{
public:

    typedef _Mesh Mesh;

    enum Version
    {
        Version1_0 = 0x100,
        LastestVersion = Version1_0
    };

public:

    /**
     * \brief Constructor
     * \param version Specify the version of the qvg file format to output.
     *   Default to the most recent version.
     */
    inline QVGWriter(const Mesh& _mesh, Version version=LastestVersion)
        : m_mesh(_mesh), m_version(version) {}

    /**
     * \brief Write `mesh` in the stream `out` as a `.qvg` file.
     * \param mesh The QMesh to save.
     * \param out The output stream.
     */
    void write(std::ostream& _out) const;

private:
    const Mesh& m_mesh;
    Version m_version;
};


#include "qvgWriter.hpp"

} // namespace Vitelotte


#endif // _QVGWRITER_H_
