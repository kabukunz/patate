/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _PATATE_COMMON_GL_UTILS_MACROS_
#define _PATATE_COMMON_GL_UTILS_MACROS_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#ifndef PATATE_ASSERT_NO_GL_ERROR
#define PATATE_ASSERT_NO_GL_ERROR()                                             \
{                                                                               \
    GLenum Error = glGetError();                                                \
                                                                                \
    if (Error != GL_NO_ERROR) {                                                 \
        printf("OpenGL error in %s:%d: 0x%x\n", __FILE__, __LINE__, Error);     \
        exit(0);                                                                \
    }                                                                           \
}
#endif

#define PATATE_FIELD_OFFSET(_struct, _field) &(static_cast<_struct*>(0)->_field)


#endif

