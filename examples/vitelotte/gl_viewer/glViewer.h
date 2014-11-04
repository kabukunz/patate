#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#ifdef WIN32

#pragma once

#ifdef _DEBUG
#pragma comment(lib, "glfw3d.lib")
#else
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glew32.liib")
#endif

#ifdef _DEBUG
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#define _CRTDBG_MAP_ALLOC

#include <crtdbg.h>
#endif // _DEBUG

#define _CRT_SECURE_NO_WARNINGS 1
#define WIN32_LEAN_AND_MEAN 1

#endif //WIN32

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <cstdio>
#include <Eigen/Dense>


namespace Vitelotte {
    template < typename _Scalar, int _Dim, int _Chan >
    class FemMesh;
    template < typename _Mesh >
    class VGMeshRenderer;
}

class GLViewer
{

public:
    struct APPINFO
    {
        char title[128];
        int windowWidth;
        int windowHeight;
        int majorVersion;
        int minorVersion;
        int samples;
        union
        {
            struct
            {
                unsigned int    fullscreen  : 1;
                unsigned int    vsync       : 1;
                unsigned int    cursor      : 1;
                unsigned int    stereo      : 1;
                unsigned int    debug       : 1;
            };
            unsigned int        all;
        } flags;
    };

    typedef float Scalar;

    typedef Vitelotte::FemMesh<Scalar, 2, 4> Mesh;
    typedef Vitelotte::VGMeshRenderer<Mesh> Renderer;

    typedef Eigen::AlignedBox<Scalar, 2> Box;

public:

    //Singleton
    static GLViewer* getAppSingleton()
    {
        if (m_pApp == NULL)
        {
            m_pApp = new GLViewer();
            if(!m_pApp)
            {
                return NULL;
            }

            if(!m_pApp->init())
            {
                fprintf(stderr, "Could not initialize the application object.\n");
                return NULL;
            }
        }
        return m_pApp;
    }

    static void killAppSingleton()
    {
        if(m_pApp != NULL)
        {
            delete m_pApp;
            m_pApp = NULL;
        }
    }

    //Flow
    void run(const std::string& filename);
    void shutdown();
    void startup(const std::string& filename);

    //Events
    void onRefresh();
    void onResize(int _w, int _h);
    void onKey(int _key, int _scancode, int _action, int _mods);
    void onMouseButton(int _button, int _action, int _mods);
    void onMouseMove(double _x, double _y);
    void onMouseWheel(double _xOffset, double _yOffset);
    void onError(int _error, const char* _description);
    void onDebugMessage(GLenum _source, GLenum _type, GLuint _id, GLenum _severity, GLsizei _length, const GLchar* _message) const;

    //Accessors
    static void getMousePosition(GLFWwindow& _window, double& _x, double& _y)
    {
        glfwGetCursorPos(&_window, &_x, &_y);
    }

    GLFWwindow* getWindow(){ return m_pWindow; }

private:
    GLViewer();
    ~GLViewer();

    bool init();
    void initVars();
    void setVsync(bool _bEnable);

    void render();

    Eigen::Vector2f screen2scene(const float& _x, const float& _y);
    Eigen::Vector2f scene2screen(const float& _x, const float& _y);

private:
    static void onRefreshCallback(GLFWwindow* /*_pWindow*/)
    {
        m_pApp->onRefresh();
    }

    static void onResizeCallback(GLFWwindow* /*_pWindow*/, int _w, int _h)
    {
        m_pApp->onResize(_w, _h);
    }

    static void onKeyCallback(GLFWwindow* /*_pWindow*/, int _key, int _scancode, int _action, int _mods)
    {
        m_pApp->onKey(_key, _scancode, _action, _mods);
    }

    static void onMouseButtonCallback(GLFWwindow* /*_pWindow*/, int _button, int _action, int _mods)
    {
        m_pApp->onMouseButton(_button, _action, _mods);
    }

    static void onMouseMoveCallback(GLFWwindow* /*_pWindow*/, double _x, double _y)
    {
        m_pApp->onMouseMove(_x, _y);
    }

    static void onMouseWheelCallback(GLFWwindow* /*_pWindow*/, double _xOffset, double _yOffset)
    {
        m_pApp->onMouseWheel(_xOffset, _yOffset);
    }

    static void errorCallback(int _error, const char* _description)
    {
        m_pApp->onError(_error, _description);
    }

    static void APIENTRY debugCallback(
            GLenum _source, GLenum _type, GLuint _id, GLenum _severity,
            GLsizei _length, const GLchar* _message, const GLvoid* _userParam)
    {
        reinterpret_cast<const GLViewer*>(_userParam)->onDebugMessage(
                    _source, _type, _id, _severity, _length, _message);
    }

private:
    static GLViewer* m_pApp;

    APPINFO             m_info;
    GLFWwindow*         m_pWindow;

    bool m_needRefresh;
    Eigen::Matrix4f m_viewMatrix;
    Eigen::Vector2f m_viewCenter;
    Eigen::Vector2f m_lastMousePos;

    float m_zoom;
    float m_pointRadius;
    float m_lineWidth;

    Mesh*     m_pQvg;
    Box       m_boundingBox;

    Renderer* m_pQMeshRenderer;

    bool m_wireframe;
    bool m_showShaderWireframe;
};

#endif
