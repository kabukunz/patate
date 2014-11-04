#ifndef _SHADER_H_
#define	_SHADER_H_

#include <cstdio>
#include <cstring>
#include <cassert>
#include <list>

//#include "../gl/glcorearb.h"

#include "macros.h"


namespace Patate {

class Shader
{
public:

    inline Shader();
    inline ~Shader();

    inline virtual bool Init();

    inline void Enable();

    inline bool AddShaderFromFile(GLenum _ShaderType, const char* _pFilename);
    inline bool AddShader(GLenum _ShaderType, const char* _pShaderText);
    inline bool Finalize();

    GLuint GetShaderId() { return m_shaderProg; }

    inline GLint GetUniformLocation(const char* _pUniformName);
    inline GLint GetProgramParam(GLint _param);
  
protected:
    GLuint m_shaderProg;
    
private:
    typedef std::list<GLuint> ShaderObjList;
    ShaderObjList m_shaderObjList;
};

//#define INVALID_UNIFORM_LOCATION 0xFFFFFFFF


#include "shader.hpp"

}

#endif

