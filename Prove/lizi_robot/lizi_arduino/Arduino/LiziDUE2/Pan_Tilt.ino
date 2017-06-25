void pan_tilt_setup() {
  //center on startup
  pan_servo.attach(PAN_SERVO_PIN);
  tilt_servo.attach(TILT_SERVO_PIN);
  pan_servo.write(PAN_CENTER);
  tilt_servo.write(TILT_CENTER);
  pan_tilt_t = millis();
}

void pan_tilt_wd() {
  unsigned long now = millis();
  if ((now - pan_tilt_t > PAN_TILT_MOVE_TIME) && (pan_tilt_moving)) {
    pan_tilt_moving = false;
    pan_servo.detach();
    tilt_servo.detach();
  }

}

void pantiltCb( const lizi::lizi_pan_tilt& msg) {

  float pan = msg.pan_angle * 180 / PI;
  float tilt = msg.tilt_angle * 180 / PI;
  if (pan > MAX_PAN) pan = MAX_PAN;
  if (pan < MIN_PAN) pan = MIN_PAN;
  if (tilt > MAX_TILT) tilt = MAX_TILT;
  if (tilt < MIN_TILT) tilt = MIN_TILT;
  tilt = tilt * 45 / 30;
  pan = pan + PAN_CENTER;
  tilt = tilt + TILT_CENTER;
  pan_tilt_t = millis();

  if ( pan_tilt_moving == false) {
    pan_servo.attach(PAN_SERVO_PIN);
    tilt_servo.attach(TILT_SERVO_PIN);
    pan_tilt_moving = true;
  }
  pan_servo.write((int)pan);
  tilt_servo.write((int)tilt);
  char tt[130];

}

