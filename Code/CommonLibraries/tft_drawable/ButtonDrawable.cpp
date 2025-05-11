#include <ButtonDrawable.hpp>


ButtonDrawable::ButtonDrawable(
        Vector2& top_left_corner, 
        Vector2& dimensions) :
    Drawable(top_left_corner, dimensions), 
    text_(""),
    text_color_(TFT_BLACK),
    text_color_clicked_(TFT_BLACK),
    text_size_(1),
    background_color_(TFT_BLACK),
    lastDrawingColor(0),
    need_redraw_(true)
{

}

void ButtonDrawable::update(
    std::string text, 
    uint16_t text_color,
    uint16_t text_color_clicked,
    uint8_t text_size,
    uint16_t background_color
)
{
    text_ = text;
    text_color_ = text_color;
    text_color_clicked_ = text_color_clicked;
    text_size_ = text_size;
    background_color_ = background_color;
}

void ButtonDrawable::draw(TFT_eSPI& target)
{
    uint16_t drawingColor = is_currently_clicked() ? TFT_WHITE : background_color_;
    if (drawingColor == lastDrawingColor && !need_redraw_)
        return;
    
    need_redraw_ = false;
    
    target.fillRect(
        top_left_corner_[0], 
        top_left_corner_[1], 
        dimensions_[0], 
        dimensions_[1],
        drawingColor);

    // text
    target.setTextSize(text_size_);
    target.setTextColor(is_currently_clicked() ? text_color_clicked_ : text_color_);
    target.setCursor(top_left_corner_[0] + 5, 
        top_left_corner_[1] + 5);
    target.print(text_.c_str());

    lastDrawingColor = drawingColor;
}

void ButtonDrawable::trigger_redraw()
{
    need_redraw_ = true;
}

void ButtonDrawable::draw_text(TFT_eSPI& target, std::string text, char line)
{
    target.setTextSize(text_size_);
    target.setTextColor(is_currently_clicked() ? text_color_clicked_ : text_color_);
    target.setCursor(top_left_corner_[0] + 5, 
        top_left_corner_[1] + 5 + 10 * line);
    target.print(text.c_str());
}