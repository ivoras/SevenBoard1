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

  DebounceButton(int pin) {
    this->pin = pin;
    this->stable_state = HIGH;
    this->stable_what = DEBOUNCE_INVALID;
    this->stable_time = this->last_stable_time = millis();
    this->stable_checked = 0;
  }

  bool update() {
    int8_t cur_state = digitalRead(this->pin);
    bool is_switch = false;
    
    if (cur_state != this->stable_state) {
      Serial.println(cur_state == HIGH ? "H" : "L");
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
        this->stable_checked = false;
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

  void handled() {
    this->stable_checked = true;
  }

  int check_depressed() {
    if (this->stable_checked || this->stable_state == LOW || this->stable_what != DEBOUNCE_LO2HI) {
      return -1;
    }
    this->stable_checked = true;
    Serial.print("yup, depressed, len=");
    Serial.println(millis()-this->last_stable_time);
    
    return 1 + millis() - this->last_stable_time;
  }

};

#endif
