#ifndef _TECHNIQUE_H_
#define	_TECHNIQUE_H_

#include <list>
#include <GL/glew.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "macros.h"

class Technique
{
public:

    inline Technique();
    inline ~Technique();

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

#define INVALID_UNIFORM_LOCATION 0xFFFFFFFF

static const char* pVSName = "VS";
static const char* pTessCSName = "TessCS";
static const char* pTessESName = "TessES";
static const char* pGSName = "GS";
static const char* pFSName = "FS";

const char* ShaderType2ShaderName(GLuint _Type)
{
    switch (_Type)
    {
    case GL_VERTEX_SHADER:
        return pVSName;
    case GL_TESS_CONTROL_SHADER:
        return pTessCSName;
    case GL_TESS_EVALUATION_SHADER:
        return pTessESName;
    case GL_GEOMETRY_SHADER:
        return pGSName;
    case GL_FRAGMENT_SHADER:
        return pFSName;
    default:
        assert(0);
    }

    return NULL;
}

Technique::Technique()
{
    m_shaderProg = 0;
}


Technique::~Technique()
{
    for (ShaderObjList::iterator it = m_shaderObjList.begin() ; it != m_shaderObjList.end() ; it++)
    {
        glDeleteShader(*it);
    }

    if (m_shaderProg != 0)
    {
        glDeleteProgram(m_shaderProg);
        m_shaderProg = 0;
    }
}


bool Technique::Init()
{
    m_shaderProg = glCreateProgram();

    if (m_shaderProg == 0)
    {
        fprintf(stderr, "Error creating shader program\n");
        return false;
    }

    return true;
}

bool Technique::AddShader(GLenum _ShaderType, const char* _pShaderText)
{
    GLuint ShaderObj = glCreateShader(_ShaderType);

    if (ShaderObj == 0)
    {
        fprintf(stderr, "Error creating shader type %d\n", _ShaderType);
        return false;
    }

    m_shaderObjList.push_back(ShaderObj);

    const GLchar* p[1];
    p[0] = _pShaderText;
    GLint Lengths[1];
    Lengths[0]= strlen(_pShaderText);
    glShaderSource(ShaderObj, 1, p, Lengths);

    glCompileShader(ShaderObj);

    GLint success;
    glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);

    if (!success)
    {
        GLchar InfoLog[1024];
        glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
        fprintf(stderr, "Error compiling %s: '%s'\n", ShaderType2ShaderName(_ShaderType), InfoLog);
        return false;
    }

    glAttachShader(m_shaderProg, ShaderObj);

    return GLCheckError();
}

bool Technique::AddShaderFromFile(GLenum _ShaderType, const char* _pFilename)
{
    FILE* fp;
    size_t filesize;
    char* pShaderText;

    fp = fopen(_pFilename, "rb");

    if(!fp)
    {
        return 0;
    }

    fseek(fp, 0, SEEK_END);
    filesize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    pShaderText = new char[filesize + 1];
    if(!pShaderText)
    {
        return 0;
    }

    fread(pShaderText, 1, filesize, fp);
    pShaderText[filesize] = 0;
    fclose(fp);

    bool res = AddShader(_ShaderType, pShaderText);

    delete [] pShaderText;

    return res;
}

bool Technique::Finalize()
{
    GLint Success = 0;
    GLchar ErrorLog[1024] = { 0 };

    glLinkProgram(m_shaderProg);

    glGetProgramiv(m_shaderProg, GL_LINK_STATUS, &Success);
    if (Success == 0)
    {
        glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
        return false;
    }

    glValidateProgram(m_shaderProg);
    glGetProgramiv(m_shaderProg, GL_VALIDATE_STATUS, &Success);
    if (!Success)
    {
        glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
        return false;
    }

    for (ShaderObjList::iterator it = m_shaderObjList.begin() ; it != m_shaderObjList.end() ; it++)
    {
        glDeleteShader(*it);
    }

    m_shaderObjList.clear();

    return GLCheckError();
}


void Technique::Enable()
{
    glUseProgram(m_shaderProg);
}


GLint Technique::GetUniformLocation(const char* _pUniformName)
{
    GLuint Location = glGetUniformLocation(m_shaderProg, _pUniformName);

    if (Location == INVALID_OGL_VALUE)
    {
        fprintf(stderr, "Warning! Unable to get the location of uniform '%s'\n", _pUniformName);
    }

    return Location;
}

GLint Technique::GetProgramParam(GLint _param)
{
    GLint ret;
    glGetProgramiv(m_shaderProg, _param, &ret);
    return ret;
}

#endif

