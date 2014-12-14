#include <cstdlib>
#include <iostream>
#include <cmath>

#ifdef EMSCRIPTEN
#  include <emscripten.h>
#endif

#include <GL/glew.h>

#include <SDL.h>

#include <Patate/vitelotte_gl.h>
#include <Patate/Vitelotte/Utils/vgMeshGL2Renderer.h>
#include <Patate/Vitelotte/Utils/mvgReader.h>

#include "orthographicCamera.h"


class GL2Viewer
{
public:
    typedef float Scalar;

    typedef Vitelotte::VGMesh<Scalar, 2, 4> Mesh;
    typedef Vitelotte::VGMeshGL2Renderer<Mesh> Renderer;

public:
    GL2Viewer()
        : m_width(800), m_height(600), m_drag(false), m_needUpdate(true),
          m_showWireframe(true)
    {}


    void init(const std::string& filename)
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
        glClearColor(.5, .5, .5, 1.f);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

#ifndef EMSCRIPTEN
        glEnable(GL_FRAMEBUFFER_SRGB);
#endif

        try
        {
            Vitelotte::readMvgFromFile(filename, m_mesh);
        }
        catch(std::runtime_error& e)
        {
            fprintf(stderr, "error reading .qvg : %s\n", e.what());
            quit(1);
        }

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

        m_renderer.initialize(&m_mesh);
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
        std::cout << "resize: " << w << ", " << h << "\n";
        glViewport(0, 0, w, h);
        glScissor(0, 0, w, h);
        m_camera.changeAspectRatio(float(w) / float(h));
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

        if(m_showWireframe)
        {
            m_wireframeShader.viewMatrix() = viewMatrix;
            m_wireframeShader.setZoom(zoom);
            m_wireframeShader.setLineWidth(1);
            m_renderer.render(m_wireframeShader);
        }

        SDL_GL_SwapBuffers();
        //SDL_GL_SwapWindow(window);
    }


    void processEvent(const SDL_Event& event)
    {
        switch(event.type)
        {
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

        m_lastMousePos = mousePos;
    }

private:
    int m_width;
    int m_height;

    Eigen::Vector2f m_lastMousePos;
    bool m_drag;
    bool m_needUpdate;

    Mesh m_mesh;

    Renderer m_renderer;
    Vitelotte::VGMeshRendererGL2DefaultShader m_defaultShader;
    Vitelotte::VGMeshRendererGL2WireframeShader m_wireframeShader;

    OrthographicCamera m_camera;
    bool m_showWireframe;
};


void app_update(GL2Viewer* viewer)
{
    viewer->update();
}


int main(int argc, char** argv)
{
    GL2Viewer* viewer = new GL2Viewer;

    viewer->init("test.mvg");

#ifdef EMSCRIPTEN
    emscripten_set_main_loop_arg((em_arg_callback_func)app_update, viewer, 0, 0);
#else
    while(true)
        app_update(viewer);
#endif

    return EXIT_SUCCESS;
}
