int ledPin = 13;
int faderPin = 3;
int colonPin = 4;
int clockPin = 5;
int latchPin = 6;
int dataPin = 7;
int buttonPin = 8;

unsigned long prev_ms = 0;
unsigned long next_second_ms = 0;
unsigned long prev_debounce_ms = 0;
int prev_button_state = LOW;
int button_state;
int mode = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(colonPin, OUTPUT);
  pinMode(faderPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  prev_ms = millis();
  next_second_ms = prev_ms;
}

void loop() {
  int ms = millis();
  
  static int seconds = 0;
  
  char tens = seconds % 10;
  char ones = seconds / 10;

  if (mode == 2) {
    tens = rand() % 10;
    ones = rand() % 10;
  }

  int interval_ms;
  switch (mode) {
    case 0:
      interval_ms = 1000;
      break;
    case 1:
      interval_ms = 100;
      break;
    case 2:
      interval_ms = 500;
      break;
  }

  if (ms >= next_second_ms) {
    unsigned char out = ((tens & 0x0f) << 4) | (ones & 0x0f);
    //printf("%02d:%02d 0x%x02x\n", tens, ones, out);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, out);  
    digitalWrite(latchPin, HIGH);

    digitalWrite(colonPin, HIGH);
    
    seconds += 1;
    if (seconds == 60) {
      seconds = 0;
    }
    next_second_ms = ms + interval_ms; // - (ms - next_second_ms);
  }

  if (ms >= next_second_ms - (interval_ms/2)) {
    digitalWrite(colonPin, LOW);
  }
  
  analogWrite(faderPin, (sin(0.002f*ms)+1.0f)*128);

  int reading = digitalRead(buttonPin);
  if (reading != prev_button_state) {
    prev_debounce_ms = ms;
  }
  
  if ((ms - prev_debounce_ms) > 50) {
    if (reading != button_state) {
      button_state = reading;

      if (button_state == HIGH) {
        mode = (mode+1)%3;
        next_second_ms = ms;
      }
      digitalWrite(ledPin, button_state);
    }
  }
  prev_button_state = reading;
  

  prev_ms = ms;
}
