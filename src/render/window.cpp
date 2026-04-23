#include "window.hpp"

#include <GLFW/glfw3.h>

#include <stdexcept>

namespace phynity::render
{

namespace
{
int glfw_ref_count = 0;
} // namespace

Window::Window(int width, int height, const char *title)
{
    if (glfw_ref_count == 0)
    {
        if (glfwInit() == GLFW_FALSE)
        {
            throw std::runtime_error("Failed to initialize GLFW");
        }
    }
    ++glfw_ref_count;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window_ == nullptr)
    {
        --glfw_ref_count;
        if (glfw_ref_count == 0)
        {
            glfwTerminate();
        }
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // VSync
}

Window::~Window()
{
    if (window_ != nullptr)
    {
        glfwDestroyWindow(window_);
    }
    --glfw_ref_count;
    if (glfw_ref_count == 0)
    {
        glfwTerminate();
    }
}

void Window::poll_events()
{
    glfwPollEvents();
}

void Window::swap_buffers()
{
    glfwSwapBuffers(window_);
}

bool Window::should_close() const
{
    return glfwWindowShouldClose(window_) != 0;
}

double Window::get_time() const
{
    return glfwGetTime();
}

void Window::get_framebuffer_size(int &width, int &height) const
{
    glfwGetFramebufferSize(window_, &width, &height);
}

void Window::begin_render_frame(float r, float g, float b, float a)
{
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

} // namespace phynity::render
