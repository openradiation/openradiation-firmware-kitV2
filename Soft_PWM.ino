// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 4000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     4000

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void set_pwm_frequency(unsigned int frequency, unsigned char pin){
  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(pin, LEDC_CHANNEL_0);
}

void set_pwm_half() {
  ledcAnalogWrite(LEDC_CHANNEL_0, 127);
}

void set_pwm_off(){
  ledcAnalogWrite(LEDC_CHANNEL_0, 0);
}
