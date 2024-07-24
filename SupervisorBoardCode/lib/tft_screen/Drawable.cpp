#include <Drawable.hpp>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define DEBOUNCE_DELAY 200
#define CLICKED_ANIMATION_DURATION 50

Drawable::Drawable(Vector2& top_left_corner, Vector2& dimensions) :
    top_left_corner_(top_left_corner),
    dimensions_(dimensions),
    state_(DrawableState::IDLE)
{

}

bool Drawable::is_currently_clicked()
{
    return millis() - last_clicked_millis < CLICKED_ANIMATION_DURATION;
}

bool Drawable::clicked(Vector2& coordinates)
{
    if (
        top_left_corner_[0] <= coordinates[0] &&
        top_left_corner_[0] + dimensions_[0] >= coordinates[0] &&
        top_left_corner_[1] <= coordinates[1] &&
        top_left_corner_[1] + dimensions_[1] >= coordinates[1]
        )
    {
        long currentMillis = millis();
        if (currentMillis - last_clicked_millis <= DEBOUNCE_DELAY)
        {
            last_clicked_millis = currentMillis;
            return false;
        }
        else
        {
            last_clicked_millis = currentMillis;
            return true;
        }
    }
    return false;
}
