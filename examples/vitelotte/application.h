#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#ifdef WIN32

#pragma once

#ifdef _DEBUG
#pragma comment(lib, "glfw3d.lib")
#else
#pragma comment(lib, "glfw3.lib")
#endif
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glew32.lib")

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

class QMesh;
class FemInMesh;
class QMeshRenderer;

//Main Application
class Application
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

public:
    //Singleton
    static Application* GetAppSingleton()
    {
        if (m_pApp == NULL)
        {
            m_pApp = new Application();
            if(!m_pApp)
            {
                return NULL;
            }

            if(!m_pApp->Init())
            {
                fprintf(stderr, "Could not initialize the application object.\n");
                return NULL;
            }
        }
        return m_pApp;
    }

    static void KillAppSingleton()
    {
        if(m_pApp != NULL)
        {
            delete m_pApp;
            m_pApp = NULL;
        }
    }

    //Flow
    void Run();
    void Shutdown();
    void Startup();

    //Events
    void OnResize(int _w, int _h);
    void OnKey(int _key, int _scancode, int _action, int _mods);
    void OnMouseButton(int _button, int _action, int _mods);
    void OnMouseMove(double _x, double _y);
    void OnMouseWheel(double _xOffset, double _yOffset);
    void OnError(int _error, const char* _description);
    void OnDebugMessage(GLenum _source, GLenum _type, GLuint _id, GLenum _severity, GLsizei _length, const GLchar* _message) const;

    //Accessors
    static void GetMousePosition(GLFWwindow& _window, double& _x, double& _y)
    {
        glfwGetCursorPos(&_window, &_x, &_y);
    }

    GLFWwindow* GetWindow(){ return m_pWindow; }

private:
    Application();
    ~Application();

    bool Init();
    void InitVars();
    void SetVsync(bool _bEnable);

    void Update(float _dTime);
    void Render(float _dTime);

    Eigen::Vector2f screen2scene(const float& _x, const float& _y);
    Eigen::Vector2f scene2screen(const float& _x, const float& _y);

private:
    static void OnResizeCallback(GLFWwindow* _pWindow, int _w, int _h)
    {
        m_pApp->OnResize(_w, _h);
    }

    static void OnKeyCallback(GLFWwindow* _pWindow, int _key, int _scancode, int _action, int _mods)
    {
        m_pApp->OnKey(_key, _scancode, _action, _mods);
    }

    static void OnMouseButtonCallback(GLFWwindow* _pWindow, int _button, int _action, int _mods)
    {
        m_pApp->OnMouseButton(_button, _action, _mods);
    }

    static void OnMouseMoveCallback(GLFWwindow* _pWindow, double _x, double _y)
    {
        m_pApp->OnMouseMove(_x, _y);
    }

    static void OnMouseWheelCallback(GLFWwindow* _pWindow, double _xOffset, double _yOffset)
    {
        m_pApp->OnMouseWheel(_xOffset, _yOffset);
    }

    static void ErrorCallback(int _error, const char* _description)
    {
        m_pApp->OnError(_error, _description);
    }

    static void APIENTRY DebugCallback(GLenum _source, GLenum _type, GLuint _id, GLenum _severity, GLsizei _length, const GLchar* _message, const GLvoid* _userParam)
    {
        reinterpret_cast<const Application *>(_userParam)->OnDebugMessage(_source, _type, _id, _severity, _length, _message);
    }

private:
    static Application* m_pApp;
    APPINFO				m_info;
    GLFWwindow*			m_pWindow;

    Eigen::Matrix4f m_viewMatrix;
    Eigen::Vector2f m_viewCenter;
    Eigen::Vector2f m_lastMousePos;

    float m_zoom;
    float m_pointRadius;
    float m_lineWidth;

    QMesh*          m_pQvg;
    QMeshRenderer*  m_pQMeshRenderer;

    bool m_wireframe;
    bool m_showShaderWireframe;

    FemInMesh* m_pTriangulation;
};

#endif
