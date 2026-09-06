#ifndef CIRCULAR_BUTTON_HPP
#define CIRCULAR_BUTTON_HPP

#include <touchgfx/widgets/Button.hpp>

class CircularButton : public touchgfx::Button
{
public:
    void setRadii(int rx, int ry)
    {
        radiusX = rx;
        radiusY = ry;
    }

    virtual void handleClickEvent(const touchgfx::ClickEvent& evt) override
    {
        int centerX = getWidth() / 2;
        int centerY = getHeight() / 2;

        int dx = evt.getX() - centerX;
        int dy = evt.getY() - centerY;

        float normalized = (float)(dx * dx) / (float)(radiusX * radiusX) +
                            (float)(dy * dy) / (float)(radiusY * radiusY);

        if (normalized <= 1.0f)
        {
            touchgfx::Button::handleClickEvent(evt);
        }
    }

private:
    int radiusX = 35;
    int radiusY = 38;
};

#endif // CIRCULAR_BUTTON_HPP
