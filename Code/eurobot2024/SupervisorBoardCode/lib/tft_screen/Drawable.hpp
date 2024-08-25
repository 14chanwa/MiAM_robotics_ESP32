#ifndef _DRAWABLE_HPP
#define _DRAWABLE_HPP

#include <Arduino.h>
#include <Types.h>
#include <Adafruit_GFX.h>    // Core graphics library

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
    virtual void draw(Adafruit_GFX& target) = 0;

protected:
    bool is_currently_clicked();

    DrawableState state_;
    long last_clicked_millis;
    Vector2 top_left_corner_;
    Vector2 dimensions_;
};

#endif