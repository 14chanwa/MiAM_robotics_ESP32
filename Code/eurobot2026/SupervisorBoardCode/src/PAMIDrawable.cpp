#include <PAMIDrawable.hpp>

#define MESSAGE_TIMEOUT 2000
#define COLOR_GREY 0x8c71

PAMIDrawable::PAMIDrawable(uint8_t pami_id, Vector2& top_left_corner, Vector2& dimensions) :
    Drawable(top_left_corner, dimensions), 
    pami_id_(pami_id),
    drawing_color_(TFT_BLACK),
    last_updated_millis_(0),
    last_message_(FullPamiReportMessage(
        false, 
        0.0, 
        PlayingSide::BLUE_SIDE, 
        0.0,
        RobotPosition(0, 0, 0),
        255))
{

}

void PAMIDrawable::update(FullPamiReportMessage& message)
{
    if (message.get_sender_id() != 255)
    {
        last_updated_millis_ = millis();
        last_message_ = message;
    }
}

void PAMIDrawable::draw(TFT_eSPI& target)
{
    
    // Drawing color is black if the message is invalid
    uint16_t drawingColor;
    bool pami_is_active = millis() - last_updated_millis_ < MESSAGE_TIMEOUT && 
        last_message_.get_sender_id() != 255;
    if (is_currently_clicked())
    {
        drawingColor = TFT_WHITE;
    }
    else if (pami_is_active)
    {
        drawingColor = (last_message_.playingSide_ == PlayingSide::BLUE_SIDE ? TFT_BLUE : TFT_YELLOW);
    }
    else
    {
        drawingColor = COLOR_GREY;
    }
    
    target.fillRect(
        top_left_corner_[0], 
        top_left_corner_[1], 
        dimensions_[0], 
        dimensions_[1],
        drawingColor);

    if (pami_is_active)
    {
        // battery reading
        target.setTextSize(1);
        target.setTextColor(TFT_BLACK, drawingColor);
        target.setCursor(top_left_corner_[0] + 10, 
            top_left_corner_[1] + 10);
        target.print(last_message_.batteryReading_);

        // current position
        target.setCursor(top_left_corner_[0] + 10, 
            top_left_corner_[1] + 30);
        target.print(last_message_.currentPosition_.x);

        target.setCursor(top_left_corner_[0] + 10, 
            top_left_corner_[1] + 40);
        target.print(last_message_.currentPosition_.y);

        target.setCursor(top_left_corner_[0] + 10, 
            top_left_corner_[1] + 50);
        target.print(last_message_.currentPosition_.theta);
    }
}