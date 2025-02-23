#ifndef _BUTTON_DRAWABLE_HPP
#define _BUTTON_DRAWABLE_HPP

#include <Drawable.hpp>
#include <string>

class ButtonDrawable : public Drawable
{
public:
    ButtonDrawable(
        Vector2& top_left_corner, 
        Vector2& dimensions);
    void draw(TFT_eSPI& target);
    void update(
        std::string text, 
        uint16_t text_color,
        uint16_t text_color_clicked,
        uint8_t text_size,
        uint16_t background_color
    );

private:
    std::string text_;
    uint16_t text_color_;
    uint16_t text_color_clicked_;
    uint8_t text_size_;
    uint16_t background_color_;
    
    uint16_t lastDrawingColor;
};

#endif