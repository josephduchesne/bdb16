#include "mechanics/Leg2D.h"

Leg2D::Leg2D(Encoder& encoder, uint8_t servo_pin, uint16_t servo_down, uint16_t servo_up)
: 
    encoder_(encoder),
    servo_pin_(servo_pin),
    servo_down_(servo_down),
    servo_up_(servo_up),
    servo_ms_(servo_down)
{

}


void Leg2D::raiseLeg(bool up) {
    servo_ms_ = up ? servo_up_ : servo_down_;
}
