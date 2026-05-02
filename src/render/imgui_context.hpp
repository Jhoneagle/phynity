#pragma once

namespace phynity::render
{

class Window;

/// RAII wrapper for Dear ImGui initialization and per-frame lifecycle.
/// Initializes ImGui with GLFW + OpenGL3 backends; tears down on destruction.
class ImGuiContext
{
public:
    explicit ImGuiContext(Window &window);
    ~ImGuiContext();

    ImGuiContext(const ImGuiContext &) = delete;
    ImGuiContext &operator=(const ImGuiContext &) = delete;

    /// Begin a new ImGui frame (call before any ImGui draw commands)
    void begin_frame();

    /// End the ImGui frame and issue draw calls (call after all ImGui draw commands)
    void end_frame();
};

} // namespace phynity::render
