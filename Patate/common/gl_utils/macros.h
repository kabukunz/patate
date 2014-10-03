#ifndef _MACROS_H_
#define	_MACROS_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define PATATE_INVALID_OGL_VALUE 0xFFFFFFFF

#define PATATE_SAFE_DELETE(p) if (p) { delete p; p = NULL; }

#define PATATE_GLExitIfError()                                                          \
{                                                                               \
    GLenum Error = glGetError();                                                \
                                                                                \
    if (Error != GL_NO_ERROR) {                                                 \
        printf("OpenGL error in %s:%d: 0x%x\n", __FILE__, __LINE__, Error);     \
        exit(0);                                                                \
    }                                                                           \
}

#define PATATE_GLCheckError() (glGetError() == GL_NO_ERROR)

#endif

