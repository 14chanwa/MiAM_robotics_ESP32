#ifndef _PAMI_DRAWABLE_HPP
#define _PAMI_DRAWABLE_HPP

#include <Drawable.hpp>
#include <Message.hpp>

class PAMIDrawable : public Drawable
{
public:
    PAMIDrawable(uint8_t pami_id, Vector2& top_left_corner, Vector2& dimensions);
    void update(PamiReportMessage& message);
    void draw(Adafruit_GFX& target);
    uint8_t pami_id_;

private:
    uint16_t drawing_color_;
    long last_updated_millis_;
    PamiReportMessage last_message_;
};

#endif