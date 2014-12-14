#ifndef _SHADER_H_
#define	_SHADER_H_

#include <cstdio>
#include <cstring>
#include <cassert>
#include <string>
#include <list>


#include "macros.h"


namespace Patate {

class Shader
{
public:
    enum Status
    {
        Uninitialized,
        NotCompiled,
        CompilationSuccessful,
        CompilationFailed
    };

public:

    inline Shader();
    inline virtual ~Shader();

    inline bool create();
    inline void destroy();

    inline void use();

    inline Status status() const { return m_status; }

    inline void setGLSLVersionHeader(const std::string& header);
    inline bool addShaderFromFile(GLenum _ShaderType, const char* _pFilename);
    inline bool addShader(GLenum _ShaderType, const char* _pShaderText);
    inline void clearShaderList();
    inline bool finalize();

    inline GLuint getShaderId() { return m_shaderProg; }

    inline GLint getUniformLocation(const char* _pUniformName);
    inline GLint getProgramParam(GLint _param);
  
protected:
    GLuint m_shaderProg;
    
private:
    typedef std::list<GLuint> ShaderObjList;

    Status m_status;
    std::string m_versionHeader;
    ShaderObjList m_shaderObjList;
};

//#define INVALID_UNIFORM_LOCATION 0xFFFFFFFF


}

#include "shader.hpp"


#endif

