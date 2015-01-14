/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_ELEMENT_BUILDER_BASE_
#define _VITELOTTE_ELEMENT_BUILDER_BASE_


#include <string>


namespace Vitelotte
{


class ElementBuilderBase
{
public:

    enum Status
    {
        StatusOK,
        StatusWarning,
        StatusError
    };

public:
    inline ElementBuilderBase() : m_status(StatusOK), m_errorString() {}
    inline Status status() const { return m_status; }
    inline const std::string& errorString() const { return m_errorString; }
    inline void resetStatus() { m_status = StatusOK; m_errorString.clear(); }

protected:
    inline void error(Status status, const std::string& errorString)
    {
        if(m_status > status)
            return;
        m_status = status;
        m_errorString = errorString;
    }

private:
    Status m_status;
    std::string m_errorString;
};


}


#endif
