#ifndef _DRAWABLE_HPP
#define _DRAWABLE_HPP

#include <Arduino.h>
#include <Types.h>
#include <TFT_eSPI.h>

enum DrawableState
{
    IDLE,
    CLICKED
};

class Drawable 
{
public:
    Drawable(Vector2& top_left_corner, Vector2& dimensions);
    bool clicked(Vector2& coordinates);
    virtual void draw(TFT_eSPI& target) = 0;

protected:
    bool is_currently_clicked();

    DrawableState state_;
    long last_clicked_millis;
    Vector2 top_left_corner_;
    Vector2 dimensions_;
};

#endif