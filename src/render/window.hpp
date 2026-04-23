#pragma once

struct GLFWwindow;

namespace phynity::render
{

/// RAII wrapper around a GLFW window.
/// Handles GLFW initialization/termination and provides basic window operations.
class Window
{
public:
    Window(int width, int height, const char *title);
    ~Window();

    Window(const Window &) = delete;
    Window &operator=(const Window &) = delete;

    /// Poll OS events (keyboard, mouse, resize, etc.)
    void poll_events();

    /// Swap front/back framebuffers
    void swap_buffers();

    /// Check if window close was requested
    bool should_close() const;

    /// Get elapsed time since GLFW init (seconds)
    double get_time() const;

    /// Get current framebuffer size in pixels
    void get_framebuffer_size(int &width, int &height) const;

    /// Get the underlying GLFW window handle
    GLFWwindow *handle() const
    {
        return window_;
    }

private:
    GLFWwindow *window_ = nullptr;
};

} // namespace phynity::render
