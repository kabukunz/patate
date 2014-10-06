
#ifndef _QVGWRITER_H_
#define _QVGWRITER_H_


#include <cassert>
#include <ostream>

#include "../Core/qMesh.h"


namespace Vitelotte
{


/**
 * \brief The QVGWriter class allow to save QMesh objects in `.qvg` files.
 *
 * \see QMesh QVGReader
 */
class QVGWriter
{
public:
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
    inline QVGWriter(Version version=LastestVersion) :
        m_version(version) {}

    /**
     * \brief Write `mesh` in the stream `out` as a `.qvg` file.
     * \param mesh The QMesh to save.
     * \param out The output stream.
     */
    void write(const QMesh& _mesh, std::ostream& _out) const;

private:
    Version m_version;
};


#include "qvgWriter.hpp"

} // namespace Vitelotte


#endif // _QVGWRITER_H_
