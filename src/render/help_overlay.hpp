#pragma once

namespace phynity::render
{

/// F1-toggled help overlay showing keyboard shortcuts.
class HelpOverlay
{
public:
    void draw();

    void toggle()
    {
        visible_ = !visible_;
    }

    bool is_visible() const
    {
        return visible_;
    }

private:
    bool visible_ = false;
};

} // namespace phynity::render
