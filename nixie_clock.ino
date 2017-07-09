#include <Wire.h>

// I2C address of the RTC
#define DS3231_I2C_ADDRESS    0x68
// I2C address of the AT24C32 EEPROM, which is NOT actually part of the DS3231, but rather the ZS-042 breakout board
#define AT24C32_I2C_ADDRESS   0x57

// TODO: Make #define's
int ledPin = 13;          // Built-in Nano v3 LED
int faderPin = 3;         // Little PWM-controlled oscillator neon lamp
int colonPin = 4;         // Blinking colon neon lamps
int clockPin = 5;         // Shift-register clock
int latchPin = 6;         // Shift-register latch (send to output)
int dataPin = 7;          // Shift-register data
int buzzerPin = 10;       // Peizzo
int modeButtonPin = 8;    // Change display mode button
int faderButtonPin = 9;   // Toggle fader lamp button

// Structure for representing time for DS3231 input and output
// Use with setDS3231time and readDS3231time functions below
struct time_t
{
  byte second;
  byte minute;
  byte hour;
  byte dayOfWeek;
  byte dayOfMonth;
  byte month;
  byte year;
};

// Structure for maintaining button debounce state
// Use with debounce_init and debounce_process below
struct debounce_t
{
  int button_pin;
  unsigned long prev_debounce_ms;
  int prev_button_state;
  int button_state;
};

enum mode_t
{
  MODE_YEAR,
  MODE_MONTH,
  MODE_DAY,
  MODE_HOUR,
  MODE_MINUTE,
  MODE_SECOND,
  MODE_OFF,
  //MODE_RANDOM,
  MODE_NUM_MODES
};

enum state_t
{
  STATE_DISPLAY,
  STATE_XFADE,
  STATE_NUM_STATES
};

// We fade from old value to new value in 32 time stratums of length 32
// This is the sequence of which time-slot to set HIGH for each stratum.
static const byte xfade_seq[] = {
  0x00, 0x10, 0x08, 0x18, 0x04, 0x14, 0x0c, 0x1c,
  0x02, 0x12, 0x0a, 0x1a, 0x06, 0x16, 0x0e, 0x1e,
  0x01, 0x11, 0x09, 0x19, 0x05, 0x15, 0x0d, 0x1d,
  0x03, 0x13, 0x0b, 0x1b, 0x07, 0x17, 0x0f, 0x1f
};

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

// Set the DS3231's time
void setDS3231time(const time_t *t)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                        // set next input to start at the seconds register
  Wire.write(decToBcd(t->second));      // set seconds
  Wire.write(decToBcd(t->minute));      // set minutes
  Wire.write(decToBcd(t->hour));        // set hours
  Wire.write(decToBcd(t->dayOfWeek));   // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(t->dayOfMonth));  // set date (1 to 31)
  Wire.write(decToBcd(t->month));       // set month
  Wire.write(decToBcd(t->year));        // set year (0 to 99)
  Wire.endTransmission();
}

// Read the DS3231's time
void readDS3231time(time_t *t)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  t->second     = bcdToDec(Wire.read() & 0x7f);
  t->minute     = bcdToDec(Wire.read());
  t->hour       = bcdToDec(Wire.read() & 0x3f);
  t->dayOfWeek  = bcdToDec(Wire.read());
  t->dayOfMonth = bcdToDec(Wire.read());
  t->month      = bcdToDec(Wire.read());
  t->year       = bcdToDec(Wire.read());
}

// TODO: DS3231 functions for setting/masking built-in alarm1 and alarm2

// Read a byte from EEPROM
uint8_t readEEPROM(uint8_t page, uint8_t entry)
{
  // byte with the actual page address
  uint16_t pageEntryAddress = (uint16_t) ((uint16_t) page << 5) | entry;
  // byte with the four MSBits of the address
  uint8_t highAddressByte  = (uint8_t) (pageEntryAddress >> 8);
  // byte with the eight LSbits of the address
  uint8_t lowAddressByte   = (uint8_t) (pageEntryAddress - ((uint16_t)highAddressByte << 8)); 
  uint8_t data; 
  
  Wire.beginTransmission(AT24C32_I2C_ADDRESS);        // Initialize the Tx buffer
  Wire.write(highAddressByte);                        // Put slave register address 1 in Tx buffer
  Wire.write(lowAddressByte);                         // Put slave register address 2 in Tx buffer
  Wire.endTransmission(false);                        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(AT24C32_I2C_ADDRESS, 1); // Read one byte from slave register address 
  delay(10);                                          // maximum write cycle time per data sheet
  data = Wire.read();                                 // Fill Rx buffer with result
  //delay(10);
  return data;
}

// Write a byte to EEPROM
void writeEEPROM(uint8_t page, uint8_t entry, uint8_t data)
{
  // Construct EEPROM address from page and entry input
  // There are 128 pages and 32 entries (bytes) per page
  // EEPROM address are 16-bit (2 byte) address where the MS four bits are zero (or don't care)
  // the next seven MS bits are the page and the last five LS bits are the entry location on the page
  uint16_t pageEntryAddress = (uint16_t) ((uint16_t) page << 5) | entry;
  // byte with the four MSBits of the address
  uint8_t highAddressByte  = (uint8_t) (pageEntryAddress >> 8);
  // byte with the eight LSbits of the address
  uint8_t lowAddressByte   = (uint8_t) (pageEntryAddress - ((uint16_t) highAddressByte << 8));
  
  Wire.beginTransmission(AT24C32_I2C_ADDRESS);    // Initialize the Tx buffer
  Wire.write(highAddressByte);              // Put slave register address 1 in Tx buffer
  Wire.write(lowAddressByte);               // Put slave register address 2 in Tx buffer
  Wire.write(data);                         // Put data in Tx buffer
  delay(10);                                // maximum write cycle time per data sheet
  Wire.endTransmission();                   // Send the Tx buffer
  //delay(10);
}

// 
// TODO: Rename to Arduino camelCase "standard"
void debounce_init(struct debounce_t *d, int pin)
{
  pinMode(pin, INPUT);
  d->button_pin = pin,
  d->prev_debounce_ms = 0;
  d->prev_button_state = LOW;
  d->button_state = LOW;
}

// TODO: Rename to Arduino camelCase "standard"
bool debounce_process(struct debounce_t *d, unsigned int ms)
{
  int reading = digitalRead(d->button_pin);
  bool rv = false;
  if (reading != d->prev_button_state) {
    d->prev_debounce_ms = ms;
  }
  
  if ((ms - d->prev_debounce_ms) > 50) {
    if (reading != d->button_state) {
      d->button_state = reading;

      if (d->button_state == HIGH) {
        rv = true;
      }
    }
  }
  d->prev_button_state = reading;
  return rv;
}


// Global State
int prev_mode = MODE_SECOND;
int mode = MODE_SECOND;
int state = STATE_DISPLAY;
bool fading = true;
bool xfade_buf[32];
unsigned int xfade_seq_idx;
unsigned int xfade_buf_idx;
debounce_t modeButton;
debounce_t faderButton;

void setup()
{
  Wire.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(colonPin, OUTPUT);
  pinMode(faderPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  debounce_init(&modeButton, modeButtonPin);
  debounce_init(&faderButton, faderButtonPin);
}

int value_for_mode(int mode, time_t *t, unsigned int ms)
{
  //static unsigned int ms_at_random; // last time we pooped out a random number
  //static int val_at_random;         // last random number pooped out
  
  switch (mode) {
    case MODE_YEAR:
      return t->year;
    case MODE_MONTH:
      return t->month;
    case MODE_DAY:
      return t->dayOfMonth;
    case MODE_HOUR:
      return t->hour;
    case MODE_MINUTE:
      return t->minute;
    case MODE_SECOND:
      return t->second;
    case MODE_OFF:
      return 100;
    /*case MODE_RANDOM:
      if (ms > ms_at_random + 250) {
        ms_at_random = ms;
        val_at_random = rand() % 100;
      }
      return val_at_random;
      */
  }
}

void loop()
{
  static byte prev_second = 0;          // Previous second from RTC
  static byte display_val = 100;        // Current Nixie display value
  static byte prev_display_val = 100;   // What did we show on Nixies last?
  static unsigned int ms_at_second = 0; // uC millis() at last RTC second rollover
  static bool colon_state = 0;
  static struct time_t t; // FIXME: static for now to avoid querying during xfade
  unsigned int ms;
  static unsigned int fading_ms = 0;

  // Get RTC and uC clock as reference
  if (state != STATE_XFADE) {
    readDS3231time(&t);
  }
  ms = millis();
  
  // Update RTC-uC sync
  if (t.second != prev_second) {
    colon_state = HIGH;
    digitalWrite(colonPin, HIGH);

    ms_at_second = ms;
  }

  // Update colon at half-second
  if (ms >= ms_at_second + 500) { // && colon_state == HIGH) {
    colon_state = LOW;
    digitalWrite(colonPin, LOW);
  }

  // figure out what to display
  switch (state) {
    case STATE_DISPLAY:
      display_val = value_for_mode(mode , &t, ms);
      break;
      
    case STATE_XFADE:
      if (xfade_buf[xfade_buf_idx]) {
        display_val = value_for_mode(mode , &t, ms);
      } else {
        display_val = value_for_mode(prev_mode , &t, ms);
      }

      xfade_buf_idx++;
      if (xfade_buf_idx == 32) {
        xfade_buf[xfade_seq[xfade_seq_idx]] = 1;
        xfade_seq_idx++;
        xfade_buf_idx = 0;
      }
      if (xfade_seq_idx == 30 /*32*/) {
        state = STATE_DISPLAY;
      }
      break;
  }

  // Now update nixies if needed
  // TODO: Remove short-circuit
  if (true || display_val != prev_display_val) {
    byte out;
    if (display_val >= 100) {
      out = 0xff;
    } else {
      byte tens = display_val / 10;
      byte ones = display_val % 10;
      out = ((ones & 0x0f) << 4) | (tens & 0x0f);
    }
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, out);  
    digitalWrite(latchPin, HIGH);

    prev_display_val = display_val;
  }

  if (fading) {
    analogWrite(faderPin, (cos(0.002f*(ms-fading_ms))+1.0f)*128);
  } else {
    analogWrite(faderPin, 0);
  }
  
  if (debounce_process(&modeButton, ms)) {
    /*
    time_t t = {second = 0, minute = 15, hour = 17,
                dayOfWeed = 5, dayOfMonth = 9, month = 3, year = 17};
    setDS3231time(&t);
    */
       
    prev_mode = mode;
    mode = (mode +1 ) % MODE_NUM_MODES;
    memset(&xfade_buf, 0, sizeof(xfade_buf));
    xfade_seq_idx = 0;
    xfade_buf_idx = 0;
    state = STATE_XFADE;
  }

  if (debounce_process(&faderButton, ms)) {
    if(fading) {
      tone(buzzerPin, 1046); 
      fading = false;
    } else {
      tone(buzzerPin, 880);
      fading = true;
    }
    fading_ms = ms;
  }
  if (ms >= fading_ms + 150) {
    noTone(buzzerPin); 
  }
  
  prev_second = t.second;
}
