#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

#include <GL/glew.h>

#include "Patate/vitelotte_gl.h"
#include "Patate/Vitelotte/Utils/mvgReader.h"
#include "Patate/Vitelotte/Utils/mvgWriter.h"

#include "../common/trackball.h"

#include "glViewer.h"


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

GLViewer* GLViewer::m_pApp = NULL;

GLViewer::GLViewer()
{
}

GLViewer::~GLViewer()
{
}

bool GLViewer::init()
{
    glfwSetErrorCallback(errorCallback);

    if(!glfwInit())
    {
        fprintf(stderr, "Could not initialize glfw.\n");
        return false;
    }

    initVars();

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

    glfwSetWindowRefreshCallback(m_pWindow, onRefreshCallback);
    glfwSetWindowSizeCallback(m_pWindow, onResizeCallback);
    glfwSetKeyCallback(m_pWindow, onKeyCallback);
    glfwSetMouseButtonCallback(m_pWindow, onMouseButtonCallback);
    glfwSetCursorPosCallback(m_pWindow, onMouseMoveCallback);
    glfwSetScrollCallback(m_pWindow, onMouseWheelCallback);

    m_info.flags.cursor ? glfwSetInputMode(m_pWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL) : glfwSetInputMode(m_pWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    glewExperimental = GL_TRUE;
    GLenum res = glewInit();
    glGetError();  // FIXME: avoid a GL error, but why glewInit fail ?
    if (res != GLEW_OK)
    {
        fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
        return false;
    }

    if (m_info.flags.debug)
    {
        if (GLEW_VERSION_4_3)
        {
            glDebugMessageCallback(debugCallback, this);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        }
        else if (glfwExtensionSupported("GL_ARB_debug_output"))
        {
            glDebugMessageCallbackARB(debugCallback, this);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
        }
    }

    PATATE_ASSERT_NO_GL_ERROR();

    return true;
}

void GLViewer::run(const std::string& filename)
{
    bool bRunning = true;

    startup(filename);

    render();
    glfwSwapBuffers(m_pWindow);
    while(bRunning)
    {
        glfwWaitEvents();

        if(m_needRefresh) {
            render();
            glfwSwapBuffers(m_pWindow);
            m_needRefresh = false;
        }

        bRunning &= (glfwGetKey(m_pWindow, GLFW_KEY_ESCAPE) == GLFW_RELEASE);
        bRunning &= (!glfwWindowShouldClose(m_pWindow));
    }

    shutdown();

    glfwTerminate();
}

void GLViewer::initVars()
{
    m_pWindow = NULL;

    strcpy(m_info.title, "Vitelotte Viewer");
    m_info.windowWidth = 800;
    m_info.windowHeight = 600;
    m_info.majorVersion = 4;
    m_info.minorVersion = 1;
    m_info.samples = 16;
    m_info.flags.all = 0;
    m_info.flags.cursor = 1;
    m_info.flags.fullscreen = 0;
#ifdef _DEBUG
    m_info.flags.debug = 1;
#endif

    m_needRefresh = false;
    m_pointRadius = 2.f;
    m_lineWidth = 1.f;

    m_pQvg = 0;
    m_pQMeshRenderer = 0;

    m_wireframe = false;
    m_showShaderWireframe = false;
}

void GLViewer::shutdown()
{
    delete m_pQvg;
    delete m_pQMeshRenderer;
        
    glfwDestroyWindow(m_pWindow);
    m_pWindow = NULL;
}

void GLViewer::startup(const std::string& filename)
{
    PATATE_ASSERT_NO_GL_ERROR();

    m_pQvg = new Mesh;

    if(!Vitelotte::readMvgFromFile(filename, *m_pQvg, Vitelotte::MVGReader<Mesh>::NO_WARN_UNKNOWN))
    {
        exit(1);
    }

    //assert(m_pQvg->isValid());
    unsigned nSingular = m_pQvg->nSingularFaces();
    std::cout << "Nb vertices : " << m_pQvg->nVertices() << "\n";
    std::cout << "Nb nodes : " << m_pQvg->nNodes() << "\n";
    //std::cout << "Nb curves : " << m_pQvg->nbCurves() << "\n";
    std::cout << "Nb triangles : " << m_pQvg->nFaces()-nSingular << "\n";
    std::cout << "Nb singular : " << nSingular << "\n";
    std::cout << std::flush;

    m_boundingBox.setEmpty();
    for(Mesh::VertexIterator vit = m_pQvg->verticesBegin();
        vit != m_pQvg->verticesEnd(); ++vit)
    {
        m_boundingBox.extend(m_pQvg->position(*vit).template head<2>());
    }

    m_trackball.setSceneRadius(m_boundingBox.sizes().maxCoeff());
    m_trackball.setSceneDistance(m_trackball.sceneRadius() * 3.);
    m_trackball.setNearFarOffsets(-m_trackball.sceneRadius() * 100.f,
                                   m_trackball.sceneRadius() * 100.f);
    m_trackball.setScreenViewport(Eigen::AlignedBox2f(
            Eigen::Vector2f(0, 0),
            Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight)));

    glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

//    glEnable(GL_FRAMEBUFFER_SRGB);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_pQMeshRenderer = new Renderer;
    m_pQMeshRenderer->updateBuffers(*m_pQvg);

    PATATE_ASSERT_NO_GL_ERROR();
}

void GLViewer::render()
{
    PATATE_ASSERT_NO_GL_ERROR();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_viewMatrix = m_trackball.computeProjectionMatrix()
                 * m_trackball.computeViewMatrix();

    float zoomFactor = 1;

    m_pQMeshRenderer->render(m_viewMatrix);

    if(m_showShaderWireframe)
    {
        m_pQMeshRenderer->renderWireframe(m_viewMatrix, zoomFactor, m_lineWidth);
    }

    PATATE_ASSERT_NO_GL_ERROR();
}

void GLViewer::onRefresh() {
    m_needRefresh = true;
}

void GLViewer::onResize(int _w, int _h)
{
    m_info.windowWidth = _w;
    m_info.windowHeight = _h;

    m_trackball.setScreenViewport(Eigen::AlignedBox2f(
            Eigen::Vector2f(0, 0),
            Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight)));
    if(glViewport != NULL)
    {
        glViewport(0, 0, _w, _h);
    }

    m_needRefresh = true;
}

void GLViewer::onKey(int _key, int /*_scancode*/, int _action, int /*_mods*/)
{
    if(_action == GLFW_PRESS || _action == GLFW_REPEAT)
    {
        switch (_key)
        {
        case GLFW_KEY_KP_ADD:
            m_lineWidth += .5f;
            m_needRefresh = true;
            break;

        case GLFW_KEY_KP_SUBTRACT:
            m_lineWidth = std::max(.5f, m_lineWidth - .5f);
            m_needRefresh = true;
            break;
        
        case GLFW_KEY_KP_MULTIPLY:
            m_pointRadius += .5f;
            m_needRefresh = true;
            break;

        case GLFW_KEY_KP_DIVIDE:
            m_pointRadius = std::max(.5f, m_pointRadius - .5f);
            m_needRefresh = true;
            break;

        case GLFW_KEY_S:
            m_showShaderWireframe = !m_showShaderWireframe;
            m_needRefresh = true;
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
            m_needRefresh = true;
            break;

        default:
            break;
        }
    }
}

void GLViewer::onMouseButton(int _button, int _action, int /*_mods*/)
{
    if(_button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if(_action == GLFW_PRESS)
        {
            double x, y;
            glfwGetCursorPos(m_pWindow, &x, &y);
            m_trackball.startTranslation(Eigen::Vector2f(x, y));
        } else if(_action == GLFW_RELEASE)
        {
            m_trackball.endTranslation();
        }
    }
    if(_button == GLFW_MOUSE_BUTTON_RIGHT && m_pQvg->nDims() != 2)
    {
        if(_action == GLFW_PRESS)
        {
            double x, y;
            glfwGetCursorPos(m_pWindow, &x, &y);
            m_trackball.startRotation(Eigen::Vector2f(x, y));
        } else if(_action == GLFW_RELEASE)
        {
            m_trackball.endRotation();
        }
    }
}

void GLViewer::onMouseMove(double x, double y)
{
    if(glfwGetMouseButton(m_pWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS &&
            m_trackball.isTranslating())
    {
        m_trackball.dragTranslate(Eigen::Vector2f(x, y));
        m_needRefresh = true;
    }
    if(glfwGetMouseButton(m_pWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS &&
            m_trackball.isRotating())
    {
        m_trackball.dragRotate(Eigen::Vector2f(x, y));
        m_needRefresh = true;
    }
}

void GLViewer::onMouseWheel(double /*_xOffset*/, double _yOffset)
{
    if((glfwGetKey(m_pWindow, GLFW_KEY_LEFT_CONTROL)  == GLFW_PRESS ||
        glfwGetKey(m_pWindow, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS) &&
            m_pQvg->nDims() != 2)
    {
        m_trackball.dollyZoom((_yOffset > 0.) ? 1.1f : 1.f/1.1f);
    }
    else if((glfwGetKey(m_pWindow, GLFW_KEY_LEFT_ALT)  == GLFW_PRESS ||
             glfwGetKey(m_pWindow, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS) &&
                 m_pQvg->nDims() != 2)
    {
        m_trackball.grow((_yOffset > 0.) ? 1.1f : 1.f/1.1f);
    }
    else
    {
        m_trackball.zoom((_yOffset > 0.) ? 1.1f : 1.f/1.1f);
    }
    m_needRefresh = true;
}

void GLViewer::onError(int /*_error*/, const char* _description)
{
    fprintf(stderr, "Error Message : %s \n", _description);
}

void GLViewer::onDebugMessage(GLenum /*_source*/, GLenum /*_type*/,
                              GLuint /*_id*/, GLenum /*_severity*/,
                              GLsizei /*_length*/, const GLchar* _message) const
{
    fprintf(stderr, "Debug Message : %s \n", _message);
}

//Eigen::Vector2f GLViewer::screen2scene(const float& _x, const float& _y)
//{
//    return (Eigen::Vector2f(_x, m_info.windowHeight - _y - 1)
//          - Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight) / 2.f) / m_zoom + m_viewCenter;
//}

//Eigen::Vector2f GLViewer::scene2screen(const float& _x, const float& _y)
//{
//    Eigen::Vector2f v = (Eigen::Vector2f(_x, _y) - m_viewCenter) * m_zoom + Eigen::Vector2f(m_info.windowWidth, m_info.windowHeight) / 2.f;

//    return Eigen::Vector2f(v.x() + .5f, m_info.windowHeight - int(v.y() + .5f) - 1);
//}


void usage(char* progName)
{
    std::cout << "usage: " << progName << " qvg_filename\n";
    exit(1);
}


int main(int argc, char** argv)
{
#ifdef WIN32
#ifdef _DEBUG
    _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif
#endif

    // TODO: Wiser argument processing
    std::string qvgFilename;
    if(argc == 2)
    {
        qvgFilename = argv[1];
    }
    else
    {
        usage(argv[0]);
    }

    GLViewer *pApp = GLViewer::getAppSingleton();

    pApp->run(std::string(argv[1]));

    if(pApp)
    {
        GLViewer::killAppSingleton();
    }
    

    return EXIT_SUCCESS;
}

