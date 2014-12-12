#ifndef _MACROS_H_
#define	_MACROS_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#define PATATE_ASSERT_NO_GL_ERROR()                                                          \
{                                                                               \
    GLenum Error = glGetError();                                                \
                                                                                \
    if (Error != GL_NO_ERROR) {                                                 \
        printf("OpenGL error in %s:%d: 0x%x\n", __FILE__, __LINE__, Error);     \
        exit(0);                                                                \
    }                                                                           \
}


#endif

