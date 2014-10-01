#include <Eigen/Geometry>
#include <iostream>

#include "application.h"
#define _RENDERING_
#include "../../Patate/vitelotte.h"

Eigen::Matrix4f orthogonalMatrix(float _left, float _right, float _bottom, float _top, float _near, float _far)
{
    float rMl = 1.0f / (_right - _left);
    float tMb = 1.0f / (_top - _bottom);
    float fMn = 1.0f / (_far - _near);

    Eigen::Matrix4f M;
    M <<
        2.0f*rMl, 0       , 0        , -(_right+_left)*rMl,
        0       , 2.0f*tMb, 0        , -(_top+_bottom)*tMb,
        0       , 0       , -2.0f*fMn, -(_far+_near)*fMn,
        0       , 0       , 0        , 1.0f;

    return M;
}

Application* Application::m_pApp = NULL;

Application::Application()
{
}

Application::~Application()
{
}

bool Application::Init()
{
    glfwSetErrorCallback(ErrorCallback);

    if(!glfwInit())
    {
        fprintf(stderr, "Could not initialize glfw.\n");
        return false;
    }

    InitVars();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, m_info.majorVersion);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, m_info.minorVersion);
#ifdef _DEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, m_info.samples);
    glfwWindowHint(GLFW_STEREO, m_info.flags.stereo ? GL_TRUE : GL_FALSE);

    //Fullscreen Mode
    if(m_info.flags.fullscreen)
    {
        if (m_info.windowWidth == 0 || m_info.windowHeight == 0)
        {
            GLFWvidmode mode;
            mode = *glfwGetVideoMode(glfwGetPrimaryMonitor());
            m_info.windowWidth = mode.width;
            m_info.windowHeight = mode.height;
        }

        m_pWindow = glfwCreateWindow(m_info.windowWidth, m_info.windowHeight, m_info.title, glfwGetPrimaryMonitor(), NULL);
        glfwSwapInterval((int)m_info.flags.vsync);
    }
    else
    {
        m_pWindow = glfwCreateWindow(m_info.windowWidth, m_info.windowHeight, m_info.title, NULL, NULL);
    }


    if (!m_pWindow)
    {
        fprintf(stderr, "Could not initialize window.\n");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_pWindow);

    glfwSetWindowSizeCallback(m_pWindow, OnResizeCallback);
    glfwSetKeyCallback(m_pWindow, OnKeyCallback);
    glfwSetMouseButtonCallback(m_pWindow, OnMouseButtonCallback);
    glfwSetCursorPosCallback(m_pWindow, OnMouseMoveCallback);
    glfwSetScrollCallback(m_pWindow, OnMouseWheelCallback);

    m_info.flags.cursor ? glfwSetInputMode(m_pWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL) : glfwSetInputMode(m_pWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    glewExperimental = GL_TRUE;
    GLenum res = glewInit();
    if (res != GLEW_OK)
    {
        fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
        return false;
    }

    if (m_info.flags.debug)
    {
        if (GLEW_VERSION_4_3)
        {
            glDebugMessageCallback(DebugCallback, this);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        }
        else if (glfwExtensionSupported("GL_ARB_debug_output"))
        {
            glDebugMessageCallbackARB(DebugCallback, this);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
        }
    }

    return true;
}

void Application::Run()
{
    bool bRunning = true;
    static float fElapsedTime = 0.f;
    static float fLastTime = 0.f;

    Startup();

    while(bRunning)
    {
        float fCurrentTime = (float)glfwGetTime();
        fElapsedTime = fCurrentTime - fLastTime;
        fLastTime = fCurrentTime;

        Update(fElapsedTime);

        Render(fElapsedTime);

        glfwSwapBuffers(m_pWindow);
        glfwPollEvents();

        bRunning &= (glfwGetKey(m_pWindow, GLFW_KEY_ESCAPE) == GLFW_RELEASE);
        bRunning &= (!glfwWindowShouldClose(m_pWindow));
    }

    Shutdown();

    glfwTerminate();
}

void Application::InitVars()
{
    m_pWindow = NULL;

    strcpy(m_info.title, "Vitelotte Viewer");
    m_info.windowWidth = 800;
    m_info.windowHeight = 600;
    m_info.majorVersion = 4;
    m_info.minorVersion = 1;
    m_info.samples = 64;
    m_info.flags.all = 0;
    m_info.flags.cursor = 1;
    m_info.flags.fullscreen = 0;
#ifdef _DEBUG
    m_info.flags.debug = 1;
#endif

    m_viewCenter = Eigen::Vector2f::Zero();
    m_zoom = 1.f;
    m_pointRadius = 2.f;
    m_lineWidth = 1.f;

    m_pQvg = 0;
    m_pQMeshRenderer = 0;

    m_wireframe = false;
    m_showShaderWireframe = false;
}

void Application::Shutdown()
{
    SAFE_DELETE(m_pQvg);
    SAFE_DELETE(m_pQMeshRenderer);
        
    glfwDestroyWindow(m_pWindow);
    m_pWindow = NULL;
}

void Application::Startup()
{
    m_pTriangulation = new FemInMesh(FemInMesh::Fraeijs);

    m_pTriangulation->read("./Data/testSurface/vertex.2.off");
    m_pTriangulation->loadConstraintMap("./Data/testSurface/constraints.cons");
    m_pTriangulation->buildElements(true, true);

    //m_pTriangulation->saveTriangulationIntoQvg("./Data/testSurface/output1.qvg");


    FemSolver solver(m_pTriangulation, 0.5);
    solver.buildMatrices(true);

    solver.solve();

    if(!solver.isSolved())
    {
        return;
    }
  
    m_pTriangulation->saveTriangulationIntoQvg("./Data/testSurface/output.qvg");

    m_pQvg = new QMesh();

    try
    {
        m_pQvg->loadQvgFromFile("./Data/testSurface/output.qvg");
    }
    catch (QvgParseError& e)
    {
        fprintf(stderr, "error reading .qvg : %s\n", e.what());
    }

    assert(m_pQvg->isValid());
    std::cout << "Nb vertices : " << m_pQvg->nbVertices() << "\n";
    std::cout << "Nb nodes : " << m_pQvg->nbNodes() << "\n";
    std::cout << "Nb curves : " << m_pQvg->nbCurves() << "\n";
    std::cout << "Nb triangles : " << m_pQvg->nbTriangles() << "\n";
    std::cout << "Nb singular : " << m_pQvg->nbSingularTriangles() << "\n";
    std::cout << std::flush;

    m_viewCenter = m_pQvg->getBoundingBox().center();
    m_zoom = 550.f / m_pQvg->getBoundingBox().sizes().maxCoeff();

    glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
    glEnable(GL_DEPTH_TEST);

    m_pQMeshRenderer = new QMeshRenderer();
    m_pQMeshRenderer->init(m_pQvg);
}

void Application::Update(float _dTime)
{}

void Application::Render(float _dTime)
{
    GLCheckError();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float xOffset = m_info.windowWidth / (2.f * m_zoom);
    float yOffset = m_info.windowHeight / (2.f * m_zoom);

    m_viewMatrix = orthogonalMatrix(
        m_viewCenter.x() - xOffset,
        m_viewCenter.x() + xOffset,
        m_viewCenter.y() - yOffset,
        m_viewCenter.y() + yOffset,
        -1, 1);

    m_pQMeshRenderer->render(m_viewMatrix, m_zoom, m_pointRadius, m_lineWidth, m_showShaderWireframe);

    GLCheckError();
}

void Application::OnResize(int _w, int _h)
{
    m_info.windowWidth = _w;
    m_info.windowHeight = _h;

    if(glViewport != NULL)
    {
        glViewport(0, 0, _w, _h);
    }
}

void Application::OnKey(int _key, int _scancode, int _action, int _mods)
{
    if(_action == GLFW_PRESS || _action == GLFW_REPEAT)
    {
        switch (_key)
        {
        case GLFW_KEY_KP_ADD:
            m_lineWidth += .5f;
            break;

        case GLFW_KEY_KP_SUBTRACT:
            m_lineWidth = std::max(.5f, m_lineWidth - .5f);
            break;
        
        case GLFW_KEY_KP_MULTIPLY:
            m_pointRadius += .5f;
            break;

        case GLFW_KEY_KP_DIVIDE:
            m_pointRadius = std::max(.5f, m_pointRadius - .5f);
            break;

        case GLFW_KEY_S:
            m_showShaderWireframe = !m_showShaderWireframe;
            std::cout << "Shader wireframe : " << (m_showShaderWireframe ? "enabled" : "disabled") << std::endl;
            break;

        case GLFW_KEY_W:
            m_wireframe = !m_wireframe;
            if (m_wireframe)
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            }
            else
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
            std::cout << "Wireframe : " << (m_wireframe ? "enabled" : "disabled") << std::endl;
            break;

        default:
            break;
        }
    }
}

void Application::OnMouseButton(int _button, int _action, int _mods)
{
    if(_button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if(_action == GLFW_PRESS)
        {
            double x, y;
            glfwGetCursorPos(m_pWindow, &x, &y);
            m_lastMousePos = screen2scene((float)x, (float)y);
        }
    }
}

void Application::OnMouseMove(double _x, double _y)
{
    if(glfwGetMouseButton(m_pWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        Eigen::Vector2f pos = screen2scene((float)_x, (float)_y);
        m_viewCenter += m_lastMousePos - pos;
    }
}

void Application::OnMouseWheel(double _xOffset, double _yOffset)
{
    m_zoom *= (_yOffset > 0.) ? 1.1f : 1.f/1.1f; 
}

void Application::OnError(int _error, const char* _description)
{
    fprintf(stderr, "Error Message : %s \n", _description);
}

void Application::OnDebugMessage(GLenum _source, GLenum _type, GLuint _id, GLenum _severity, GLsizei _length, const GLchar* _message) const
{
    fprintf(stderr, "Debug Message : %s \n", _message);
}

Eigen::Vector2f Application::screen2scene(const float& _x, const float& _y)
{
    return (Eigen::Vector2f(_x, m_info.windowHeight - _y - 1) - Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight) / 2.f) / m_zoom + m_viewCenter;
}

Eigen::Vector2f Application::scene2screen(const float& _x, const float& _y)
{
    Eigen::Vector2f v = (Eigen::Vector2f(_x, _y) - m_viewCenter) * m_zoom + Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight) / 2.f;

    return Eigen::Vector2f(v.x() + .5f, m_info.windowHeight - int(v.y() + .5f) - 1);
}

int main(int argc, char** argv)
{
#ifdef WIN32
#ifdef _DEBUG
    _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif
#endif
    fprintf(stderr, "*************** Let the program begin ***************\n");

    Application *pApp = Application::GetAppSingleton();

    pApp->Run();

    if(pApp)
    {
        Application::KillAppSingleton();
    }
    

    fprintf(stderr, "*************** Let it end ***************\n");

    return EXIT_SUCCESS;
}

