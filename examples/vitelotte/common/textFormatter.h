#ifndef _EXAMPLES_VITELOTTE_COMMON_TEXT_FORMATTER_
#define _EXAMPLES_VITELOTTE_COMMON_TEXT_FORMATTER_


#include <ostream>
#include <string>


void format(std::ostream& out, const char* text, unsigned width,
            unsigned indent = 0, int firstIndent = -1);


#endif
