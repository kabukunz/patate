#include "vgMeshWithCurvesReader.h"


VGMeshWithCurveReader::VGMeshWithCurveReader()
{

}


bool VGMeshWithCurveReader::parseDefinition(const std::string& spec,
                                            std::istream& def)
{
    if(spec == "dc") {
        parseDc(def);
        return true;
    }

    return Base::parseDefinition(spec, def);
}

void VGMeshWithCurveReader::parseDc(std::istream& def)
{

}
