/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <cassert>

#include "textFormatter.h"


void format(std::ostream& out, const char* text, unsigned width,
            unsigned indent, int firstIndent)
{
    assert(indent < width);

    if(firstIndent < 0) firstIndent = indent;

    int count = 0;
    const char* lineBegin = text;
    unsigned nIndent = firstIndent;
    while(*lineBegin != '\0')
    {
        ++count;
        unsigned col = nIndent;
        const char* lastBreak = lineBegin;
        const char* pos       = lineBegin;
        while(*pos != '\0')
        {
            while(col < width && *pos != '\0' && !std::isspace(*pos)) { ++pos; ++col; }
//            out << "#> " << col << ": ";
//            out.write(lineBegin, pos - lineBegin);
//            out << "\n";
            if(col == width)
            {
                if(lastBreak == lineBegin)
                {
                    lastBreak = lineBegin + width - nIndent;
                }
                break;
            }
            else if(*pos == '\n' || *pos == '\0')
            {
                lastBreak = pos;
                break;
            }
            lastBreak = pos;
            ++pos; ++col;
        }
        for(unsigned i = 0; i < nIndent; ++i) out.put(' ');
        out.write(lineBegin, lastBreak - lineBegin);
        out.put('\n');
        if(*lastBreak == '\n') ++lastBreak;
        while(*lastBreak == ' ' || *lastBreak == '\t') ++lastBreak;
        lineBegin = lastBreak;
        nIndent = indent;
    }
}
