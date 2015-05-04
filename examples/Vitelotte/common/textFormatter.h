/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_TEXT_FORMATTER_
#define _EXAMPLES_VITELOTTE_COMMON_TEXT_FORMATTER_


#include <ostream>
#include <string>


void format(std::ostream& out, const char* text, unsigned width,
            unsigned indent = 0, int firstIndent = -1);


#endif
