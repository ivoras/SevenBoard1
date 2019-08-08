#ifndef _DEBOUNCE_H_
#define _DEBOUNCE_H_

#define DEBOUNCE_MILLIS 250
#define DEBOUNCE_INVALID 0
#define DEBOUNCE_HI2LO 1
#define DEBOUNCE_LO2HI 2

// with pullups
class DebounceButton {

public:
  int8_t pin;
  bool stable_checked;
  int8_t stable_state; 
  int8_t stable_what;
  uint32_t stable_time;
  uint32_t last_stable_time;
  bool ignore_next_event;

  DebounceButton(int pin) {
    this->pin = pin;
    this->stable_state = HIGH;
    this->stable_what = DEBOUNCE_INVALID;
    this->stable_time = this->last_stable_time = millis();
    this->stable_checked = 0;
    this->ignore_next_event = false;
  }

  bool update() {
    int8_t cur_state = digitalRead(this->pin);
    bool is_switch = false;
    
    if (cur_state != this->stable_state) {
      if (millis() - this->stable_time >= DEBOUNCE_MILLIS) {
        // recognize this as a button press
        if (cur_state == LOW) {
          this->stable_what = DEBOUNCE_HI2LO;
        } else {
          this->stable_what = DEBOUNCE_LO2HI;
        }
        this->last_stable_time = this->stable_time;
        this->stable_time = millis();
        this->stable_state = cur_state;
        if (!ignore_next_event) {
          this->stable_checked = false;
        } else {
          this->stable_checked = true;
          this->ignore_next_event = false;
        }
        is_switch = true;
      }
    }
    return is_switch;
  }

  bool pressed() {
    if (this->stable_checked) {
      return false;
    }
    if (this->stable_what == DEBOUNCE_HI2LO) {
      this->stable_checked = true;
      return true;
    }
    return false;
  }

  bool pressed_for(uint32_t ms) {
    if (this->stable_checked || this->stable_state != LOW) {
      return false;
    }
    if (millis() - this->stable_time >= ms) {
      this->stable_checked = true;
      this->ignore_next_event = true; // don't register the depress for a long press
      return true;
    }
    return false;
  }

  void handled() {
    this->stable_checked = true;
  }

  int check_depressed() {
    if (this->stable_checked || this->stable_state == LOW || this->stable_what != DEBOUNCE_LO2HI) {
      return -1;
    }
    this->stable_checked = true;
    return 1 + millis() - this->last_stable_time;
  }

};

#endif
