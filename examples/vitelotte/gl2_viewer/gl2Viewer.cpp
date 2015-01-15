#include <cstdlib>
#include <iostream>
#include <cmath>

#ifdef EMSCRIPTEN
#  include <emscripten.h>
#  include <emscripten/bind.h>
#endif

#include <GL/glew.h>

#include <SDL.h>

#include <Patate/vitelotte.h>
#include <Patate/vitelotte_gl.h>
#include <Patate/vitelotte_io.h>
#include <Patate/Vitelotte/Utils/vgMeshGL2Renderer.h>

#include "../common/orthographicCamera.h"
#include "../common/vgNodeRenderer.h"


class GL2Viewer
{
public:
    typedef float Scalar;

    typedef Vitelotte::VGMesh<Scalar, 2, 4> Mesh;
    typedef Vitelotte::VGMeshGL2Renderer<Mesh> Renderer;

    typedef Vitelotte::FVElementBuilder<Mesh, double> FVElementBase;
    typedef Vitelotte::SingularElementDecorator<FVElementBase> FVElement;
    typedef Vitelotte::FemSolver<Mesh, FVElement> FVSolver;


public:
    static GL2Viewer* getInstance()
    {
        return m_instance;
    }

public:
    GL2Viewer()
        : m_width(800), m_height(600), m_drag(false), m_needUpdate(true),
          m_showWireframe(true)
    {
        assert(!m_instance);
        m_instance = this;
    }


    void init()
    {
        if(SDL_Init(SDL_INIT_VIDEO) != 0)
        {
            std::cerr << "Error " << SDL_GetError() << ": Failed to initialize SDL2. Aborting.\n";
            quit(EXIT_FAILURE);
        }

        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        SDL_WM_SetCaption("OpenGL 2 mvg viewer", "");
        SDL_Surface* screen = SDL_SetVideoMode(m_width, m_height, 0, SDL_OPENGL /*| SDL_RESIZABLE*/);
        if(!screen)
        {
            std::cerr << "Error " << SDL_GetError() << ": Failed to create a screen. Aborting.\n";
            quit(EXIT_FAILURE);
        }

    //    SDL_Window* window = SDL_CreateWindow(
    //                "gl2_viewer",
    //                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
    //                800, 600,
    //                SDL_WINDOW_OPENGL);
    //    if(!window)
    //    {
    //        std::cerr << "Error " << SDL_GetError() << ": Failed to create a window. Aborting.\n";
    //        return EXIT_FAILURE;
    //    }

    //    SDL_GLContext context = SDL_GL_CreateContext(window);
    //    if(!context)
    //    {
    //        std::cerr << "Error " << SDL_GetError() << ": Failed to create OpenGL context. Aborting.\n";
    //        return EXIT_FAILURE;
    //    }

#ifndef EMSCRIPTEN
        GLenum res = glewInit();
        glGetError();  // FIXME: avoid a GL error, but why glewInit fail ?
        if (res != GLEW_OK)
        {
            fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
            quit(EXIT_FAILURE);
        }

        if(!GLEW_VERSION_3_0)
        {
            std::cerr << "OpenGL 3.0 not supported. Aborting.\n";
            abort();
        }
#endif

        glViewport(0, 0, m_width, m_height);
        glClearColor(0.f, 0.f, 0.f, 1.f);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

#ifndef EMSCRIPTEN
        glEnable(GL_FRAMEBUFFER_SRGB);
#endif
    }


    void loadMesh(const std::string& filename)
    {
        try
        {
            Vitelotte::readMvgFromFile(filename, m_baseMesh);
        }
        catch(std::runtime_error& e)
        {
            fprintf(stderr, "error reading .qvg : %s\n", e.what());
            quit(1);
        }


        m_mesh = m_baseMesh;

        unsigned time = SDL_GetTicks();
        std::cout << "Solving '" << filename << "'... ";
        std::cout.flush();

        m_mesh.setAttributes(Mesh::FV_FLAGS);
        m_mesh.finalize();

        FVSolver solver(&m_mesh);
        solver.build();
        solver.sort();
        solver.solve();

        if(solver.status() != FVElement::StatusOK)
        {
            std::cerr << "Solver error: " << solver.errorString() << "\n";
        }
        if(!solver.isSolved())
        {
            std::cerr << "Solve failed./n";
        }

        time = SDL_GetTicks() - time;
        std::cout << time << "ms" << std::endl;

        centerView();
        m_renderer.initialize(&m_mesh);

        m_needUpdate = true;
    }


    void centerView()
    {
        OrthographicCamera::ViewBox box;
        for(Mesh::VertexIterator vit = m_mesh.verticesBegin();
            vit != m_mesh.verticesEnd(); ++vit)
        {
            box.extend((Eigen::Vector3f() << m_mesh.position(*vit), 0).finished());
        }
        box.min()[2] = -1;
        box.max()[2] = 1;
        m_camera.setViewBox(box);
        m_camera.changeAspectRatio(float(m_width) / float(m_height));

        m_needUpdate = true;
    }


    bool showWireframe() const
    {
        return m_showWireframe;
    }


    void setShowWireframe(bool showWireframe)
    {
        if(showWireframe != m_showWireframe)
        {
            m_showWireframe = showWireframe;
            m_needUpdate = true;
        }
    }


    void quit(int return_status = EXIT_SUCCESS)
    {
        //    SDL_GL_DeleteContext(context);
        //    SDL_DestroyWindow(window);

        SDL_Quit();
        delete this;

        exit(return_status);
    }


    void resize(int w, int h)
    {
        glViewport(0, 0, w, h);
        m_camera.changeAspectRatio(float(w) / float(h));

        m_width = w;
        m_height = h;

        m_needUpdate = true;
    }


    void update()
    {
        SDL_Event event;
        while(SDL_PollEvent(&event))
            processEvent(event);

        if(m_needUpdate)
            render();

#ifndef EMSCRIPTEN
        // Block until something happen.
        if(SDL_WaitEvent(&event))
            processEvent(event);
#endif
    }


    void render()
    {
        m_needUpdate = false;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Eigen::Matrix4f viewMatrix = m_camera.projectionMatrix();
        float zoom = m_width * viewMatrix(0, 0) / 2.f;

        m_defaultShader.viewMatrix() = viewMatrix;
        m_renderer.render(m_defaultShader);

#ifdef EMSCRIPTEN
        if(m_showWireframe)
        {
            m_wireframeShader.viewMatrix() = viewMatrix;
            m_wireframeShader.setZoom(zoom);
            m_wireframeShader.setLineWidth(.5);
            m_wireframeShader.setWireframeColor(Eigen::Vector4f(0, 0, 0, .25));
            m_renderer.render(m_wireframeShader);
        }
#else
        if(m_showWireframe)
        {
            m_nodeRenderer.update(m_baseMesh, zoom);
            m_nodeRenderer.render(viewMatrix, Eigen::Vector2f(m_width, m_height));
        }
#endif

        SDL_GL_SwapBuffers();
        //SDL_GL_SwapWindow(window);
    }


    void processEvent(const SDL_Event& event)
    {
        switch(event.type)
        {
        case SDL_KEYDOWN:
            keyPressEvent(event.key);
            break;
        case SDL_KEYUP:
            keyReleaseEvent(event.key);
            break;
        case SDL_MOUSEBUTTONDOWN:
            mousePressEvent(event.button);
            break;
        case SDL_MOUSEBUTTONUP:
            mouseReleaseEvent(event.button);
            break;
        case SDL_MOUSEMOTION:
            mouseMoveEvent(event.motion);
            break;
        case SDL_VIDEORESIZE:
            resize(event.resize.w, event.resize.h);
            m_width = event.resize.w;
            m_height = event.resize.h;
            break;
        case SDL_QUIT:
            quit();
            break;
        }
    }


    void keyPressEvent(const SDL_KeyboardEvent& event)
    {
        switch(event.keysym.sym)
        {
        case SDLK_w:
            setShowWireframe(!m_showWireframe);
            break;
        default:
            break;
        }
    }


    void keyReleaseEvent(const SDL_KeyboardEvent& /*event*/)
    {

    }


    void mousePressEvent(const SDL_MouseButtonEvent& event)
    {
        Eigen::Vector2f mousePos(
                    float(event.x*2) / m_width - 1.f,
                    float(-event.y*2) / m_height + 1.f);

        if(event.button == SDL_BUTTON_LEFT)
        {
            m_drag = true;
        }

        m_lastMousePos = mousePos;
    }


    void mouseReleaseEvent(const SDL_MouseButtonEvent& event)
    {
        Eigen::Vector2f mousePos(
                    float(event.x*2) / m_width - 1.f,
                    float(-event.y*2) / m_height + 1.f);

        switch(event.button) {
        case SDL_BUTTON_LEFT:
            if(m_drag)
            {
                m_camera.normalizedTranslate(mousePos - m_lastMousePos);
                m_needUpdate = true;
                m_drag = false;
            }
            break;
        case SDL_BUTTON_WHEELUP:
            m_camera.normalizedZoom(mousePos, 1.1f);
            m_needUpdate = true;
            break;
        case SDL_BUTTON_WHEELDOWN:
            m_camera.normalizedZoom(mousePos, 1.f/1.1f);
            m_needUpdate = true;
            break;
        }

        m_lastMousePos = mousePos;
    }


    void mouseMoveEvent(const SDL_MouseMotionEvent& event)
    {
        Eigen::Vector2f mousePos(
                    float(event.x*2) / m_width - 1.f,
                    float(-event.y*2) / m_height + 1.f);

        if(m_drag)
        {
            m_camera.normalizedTranslate(mousePos - m_lastMousePos);
            m_needUpdate = true;
        }

        // Allow node picking.
//        Eigen::Vector2f p = m_camera.normalizedToCamera(mousePos);
//        Mesh::Node hn = m_nodeRenderer.highlightedNode();
//        Mesh::Node pick = m_nodeRenderer.pickNode(p);
//        if(pick != hn)
//        {
//            m_nodeRenderer.setHighlightedNode(pick);
//            m_needUpdate = true;
//        }

        m_lastMousePos = mousePos;
    }

private:
    static GL2Viewer* m_instance;

private:
    int m_width;
    int m_height;

    Eigen::Vector2f m_lastMousePos;
    bool m_drag;
    bool m_needUpdate;

    Mesh m_baseMesh;
    Mesh m_mesh;

    Renderer m_renderer;
    Vitelotte::VGMeshRendererGL2DefaultShader m_defaultShader;
    Vitelotte::VGMeshRendererGL2WireframeShader m_wireframeShader;

    VGNodeRenderer m_nodeRenderer;

    OrthographicCamera m_camera;
    bool m_showWireframe;
};

GL2Viewer* GL2Viewer::m_instance = 0;


#ifdef EMSCRIPTEN
EMSCRIPTEN_BINDINGS(gl2ViewerBinding) {
  emscripten::class_<GL2Viewer>("GL2Viewer")
    .function("resize", &GL2Viewer::resize)
    .function("loadMesh", &GL2Viewer::loadMesh)
    .function("centerView", &GL2Viewer::centerView)
    .function("showWireframe", &GL2Viewer::showWireframe)
    .function("setShowWireframe", &GL2Viewer::setShowWireframe)
    .class_function("getInstance", &GL2Viewer::getInstance, emscripten::allow_raw_pointers())
    ;
}
#endif


void app_update(GL2Viewer* viewer)
{
    viewer->update();
}


int main(int argc, char** argv)
{
    GL2Viewer* viewer = new GL2Viewer;

    viewer->init();
//    viewer->centerView();

#ifdef EMSCRIPTEN
    emscripten_set_main_loop_arg((em_arg_callback_func)app_update, viewer, 0, 0);
#else
    if(argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " FILENAME\n";
        return 1;
    }
    viewer->loadMesh(argv[1]);

    while(true)
        app_update(viewer);

    delete viewer;
#endif

    return EXIT_SUCCESS;
}
