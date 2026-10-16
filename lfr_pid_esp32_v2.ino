/*
 * PID Line Follower Robot - ESP32 (38 pin)  -- v2
 * Driver : TB6612FNG   Display : SSD1306 128x64 I2C   Input : KY-040 rotary encoder
 * Sensors: 5x digital IR (OUT1 = leftmost ... OUT5 = rightmost)
 *
 * Libraries (Library Manager): "Adafruit SSD1306", "Adafruit GFX Library"
 *
 * Menu (rotate = move cursor, click = act):
 *   Speed / Kp / Kd  -> click to select (shows *), rotate to change, click to confirm
 *   OnLine: HIGH/LOW -> click toggles which pin level means "sensor is ON the line"
 *   Sensors          -> live view, motors off. Put car on the line and check boxes
 *   Motor test       -> left wheel forward, then right wheel forward
 *   START            -> run. In RUN mode a click stops the car and returns to menu
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// ---------------- PINS ----------------
#define ENC_CLK 16
#define ENC_DT  17
#define ENC_SW  18

#define OLED_SDA 21
#define OLED_SCL 22

const uint8_t SENS_PINS[5] = {34, 35, 32, 33, 4};   // OUT1..OUT5 (left -> right)

// TB6612FNG  (A = left motor, B = right motor)
#define PWMA 13
#define AIN1 12
#define AIN2 14
#define PWMB 25
#define BIN1 27
#define BIN2 26
// STBY of the TB6612 must be tied to 3.3V

// ---------------- CONFIG ----------------
#define LEFT_INVERT   false        // flip if left motor spins backwards in Motor test
#define RIGHT_INVERT  false        // flip if right motor spins backwards in Motor test
#define ENC_REVERSE   false        // flip if encoder turns the wrong way
#define DEBUG_SERIAL  1            // 1 = print sensors/error/motor values on Serial (115200)
#define SAVE_SETTINGS 1            // 1 = remember settings in flash

const int8_t SENSOR_WEIGHT[5] = {-4, -2, 0, 2, 4};

#define LOOP_US        5000        // PID period (5 ms)
#define LOST_GRACE_MS  40          // hold last command this long before "lost" recovery starts
#define SEARCH_MS      700         // spin time toward last-known side (then sweeps back 2x this)
#define SEARCH_SPEED   90          // pivot speed while searching (raise if wheels don't turn)

const bool ALL_ON_STOPS = false;   // true = treat 5/5 sensors as finish box and stop
#define ALL_ON_STOP_MS 150
#define ALL_ON_MAX_MS  400         // 5/5 for longer than this = fault (wrong polarity?) -> stop

// ---------------- LIVE SETTINGS ----------------
int     speedVal    = 60;
float   kp          = 12.0f;
float   kd          = 18.0f;
uint8_t activeLevel = LOW;         // pin level that means "sensor is on the line"

// ---------------- GLOBALS ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences prefs;

enum Mode { MODE_EDIT, MODE_RUN, MODE_SENSOR };
Mode mode = MODE_EDIT;

#define N_ITEMS 7                  // 0 speed,1 kp,2 kd,3 level,4 sensors,5 motor test,6 start
int8_t cursorPos = 0;
bool   adjusting = false;
const char* banner = nullptr;

float    prevErr    = 0;
int8_t   lastSide   = 0;           // -1 line left via the left side, +1 via the right side
bool     lostActive = false;
uint32_t lostStart  = 0;
uint32_t allOnSince = 0;

// ---------------- PWM wrapper (ESP32 core 2.x and 3.x) ----------------
#define PWM_FREQ 20000
#define PWM_RES  8
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmInit()                         { ledcAttach(PWMA, PWM_FREQ, PWM_RES); ledcAttach(PWMB, PWM_FREQ, PWM_RES); }
  void pwmWrite(uint8_t pin, uint8_t d)  { ledcWrite(pin, d); }
#else
  void pwmInit() {
    ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, 1);
  }
  void pwmWrite(uint8_t pin, uint8_t d)  { ledcWrite(pin == PWMA ? 0 : 1, d); }
#endif

// ---------------- MOTORS ----------------
void setMotor(bool left, int spd) {          // -255..255, 0 = short brake
  spd = constrain(spd, -255, 255);
  if (left  && LEFT_INVERT)  spd = -spd;
  if (!left && RIGHT_INVERT) spd = -spd;

  uint8_t in1 = left ? AIN1 : BIN1;
  uint8_t in2 = left ? AIN2 : BIN2;
  uint8_t pwm = left ? PWMA : PWMB;

  if (spd > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (spd < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); }
  pwmWrite(pwm, abs(spd));
}

void drive(int l, int r) { setMotor(true, l); setMotor(false, r); }
void stopMotors()        { drive(0, 0); }
void spin(int8_t dir)    { drive(dir * SEARCH_SPEED, -dir * SEARCH_SPEED); }   // -1 left, +1 right

// ---------------- SENSORS ----------------
// returns bitmask (bit0 = OUT1 ... bit4 = OUT5), 1 = on the line (after polarity)
uint8_t readSensors(uint8_t &cnt, int &sum) {
  uint8_t mask = 0;
  cnt = 0; sum = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(SENS_PINS[i]) == activeLevel) {
      mask |= (1 << i);
      cnt++;
      sum += SENSOR_WEIGHT[i];
    }
  }
  return mask;
}

// ---------------- ENCODER / BUTTON ----------------
int8_t readEncoder() {
  static const int8_t TBL[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  static uint8_t st = 0;
  static int8_t acc = 0;
  st = ((st << 2) | (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT)) & 0x0F;
  acc += TBL[st];
  int8_t out = 0;
  if (acc >= 4)       { acc = 0; out =  1; }
  else if (acc <= -4) { acc = 0; out = -1; }
  return ENC_REVERSE ? -out : out;
}

bool buttonClicked() {
  static bool last = HIGH;
  static uint32_t t = 0;
  bool now = digitalRead(ENC_SW);
  if (now != last && millis() - t > 40) {
    t = millis();
    last = now;
    if (now == LOW) return true;
  }
  return false;
}

// ---------------- SETTINGS STORAGE ----------------
void saveSettings() {
#if SAVE_SETTINGS
  prefs.putInt("spd", speedVal);
  prefs.putFloat("kp", kp);
  prefs.putFloat("kd", kd);
  prefs.putUChar("act", activeLevel);
#endif
}

// ---------------- DISPLAY ----------------
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(banner ? banner : "LFR SETTINGS");

  int first = constrain(cursorPos - 2, 0, N_ITEMS - 5);
  for (int r = 0; r < 5; r++) {
    int i = first + r;
    int y = 12 + r * 10;
    display.setCursor(0, y);
    display.print(i == cursorPos ? (adjusting ? "*" : ">") : " ");
    display.setCursor(10, y);
    switch (i) {
      case 0: display.print("Speed : "); display.print(speedVal); break;
      case 1: display.print("Kp    : "); display.print(kp, 1);    break;
      case 2: display.print("Kd    : "); display.print(kd, 1);    break;
      case 3: display.print("OnLine: "); display.print(activeLevel == HIGH ? "HIGH" : "LOW"); break;
      case 4: display.print("Sensors view");  break;
      case 5: display.print("Motor test");    break;
      case 6: display.print("[ START ]");     break;
    }
  }
  display.display();
}

void drawRun() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("RUNNING");
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("Spd:"); display.print(speedVal);
  display.print(" Kp:");  display.print(kp, 1);
  display.setCursor(0, 36);
  display.print("Kd:");   display.print(kd, 1);
  display.setCursor(0, 54);
  display.print("click = STOP");
  display.display();
}

void drawSensors() {
  uint8_t cnt; int sum;
  readSensors(cnt, sum);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("SENSORS    click=exit");
  for (int i = 0; i < 5; i++) {
    int x = 4 + i * 25;
    bool on = (digitalRead(SENS_PINS[i]) == activeLevel);
    if (on) display.fillRect(x, 12, 20, 20, SSD1306_WHITE);
    else    display.drawRect(x, 12, 20, 20, SSD1306_WHITE);
    display.setCursor(x + 7, 37);
    display.print(digitalRead(SENS_PINS[i]));          // raw pin level
  }
  display.setCursor(0, 50);
  if (cnt == 0)      display.print("LOST (0 on line)");
  else if (cnt == 5) display.print("ALL 5 on line");
  else { display.print("err: "); display.print((float)sum / cnt, 1); }
  display.display();
}

// ---------------- TESTS ----------------
void motorTest() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("MOTOR TEST");
  display.setCursor(0, 16); display.print("LEFT wheel -> FORWARD");
  display.display();
  setMotor(true, 100); setMotor(false, 0);
  delay(800);
  stopMotors(); delay(400);

  display.clearDisplay();
  display.setCursor(0, 0);  display.print("MOTOR TEST");
  display.setCursor(0, 16); display.print("RIGHT wheel -> FORWARD");
  display.display();
  setMotor(true, 0); setMotor(false, 100);
  delay(800);
  stopMotors();
}

// ---------------- RUN CONTROL ----------------
void startRun() {
  saveSettings();
  adjusting = false;
  banner = nullptr;
  drawRun();
  delay(500);
  prevErr = 0;
  lostActive = false;
  allOnSince = 0;
  lastSide = 0;
  mode = MODE_RUN;
}

void enterEdit(const char* msg) {
  stopMotors();
  mode = MODE_EDIT;
  adjusting = false;
  banner = msg;
  drawMenu();
}

void handleLost() {
  uint32_t now = millis();
  if (!lostActive) { lostActive = true; lostStart = now; }
  uint32_t t = now - lostStart;

  if (t < LOST_GRACE_MS) return;            // tiny gap: keep previous command
  t -= LOST_GRACE_MS;

  int8_t dir = (lastSide == 0) ? 1 : lastSide;
  if (t < SEARCH_MS)            spin(dir);          // back toward the side the line left from
  else if (t < SEARCH_MS * 3UL) spin(-dir);         // sweep the other way
  else                          enterEdit("LINE LOST");
}

void runStep() {
  static uint32_t lastTick = 0;
  if (micros() - lastTick < LOOP_US) return;
  lastTick = micros();

  uint8_t cnt; int sum;
  uint8_t mask = readSensors(cnt, sum);

  // ---- lost ----
  if (cnt == 0) { handleLost(); return; }

  float err = (float)sum / cnt;             // -4 (line far left) ... +4 (line far right)
  if (lostActive) { lostActive = false; prevErr = err; }

  // remember which side the line is drifting off to (only strong readings count)
  if (err <= -2)      lastSide = -1;
  else if (err >= 2)  lastSide = 1;

  // ---- all 5 on line ----
  if (cnt == 5) {
    if (allOnSince == 0) allOnSince = millis();
    uint32_t d = millis() - allOnSince;
    if (ALL_ON_STOPS && d > ALL_ON_STOP_MS) { enterEdit("FINISH"); return; }
    if (d > ALL_ON_MAX_MS)                  { enterEdit("ALL ON: CHECK LEVEL"); return; }
    prevErr = 0;
    drive(speedVal, speedVal);
    return;
  }
  allOnSince = 0;

  // ---- PID ----
  float corr = kp * err + kd * (err - prevErr);
  prevErr = err;
  int l = (int)(speedVal + corr);
  int r = (int)(speedVal - corr);
  drive(l, r);

#if DEBUG_SERIAL
  static uint32_t dbg = 0;
  if (millis() - dbg > 100) {
    dbg = millis();
    Serial.printf("S:%d%d%d%d%d err:%.1f corr:%.1f L:%d R:%d\n",
                  mask & 1, (mask >> 1) & 1, (mask >> 2) & 1, (mask >> 3) & 1, (mask >> 4) & 1,
                  err, corr, l, r);
  }
#endif
}

// ---------------- SETUP / LOOP ----------------
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  for (int i = 0; i < 5; i++) pinMode(SENS_PINS[i], INPUT);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pwmInit();
  stopMotors();

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

#if SAVE_SETTINGS
  prefs.begin("lfr", false);
  speedVal    = prefs.getInt("spd", 60);
  kp          = prefs.getFloat("kp", 12.0f);
  kd          = prefs.getFloat("kd", 18.0f);
  activeLevel = prefs.getUChar("act", LOW);
#endif

  drawMenu();
}

void loop() {
  int8_t rot   = readEncoder();
  bool   click = buttonClicked();

  if (mode == MODE_RUN) {
    if (click) { enterEdit("STOPPED"); return; }
    runStep();
    return;
  }

  if (mode == MODE_SENSOR) {
    if (click) { enterEdit(nullptr); return; }
    static uint32_t t = 0;
    if (millis() - t > 50) { t = millis(); drawSensors(); }
    return;
  }

  // ---- EDIT mode ----
  if (rot) {
    if (adjusting) {
      if (cursorPos == 0)      speedVal = constrain(speedVal + rot, 0, 255);
      else if (cursorPos == 1) kp = constrain(kp + rot * 0.5f, 0.0f, 100.0f);
      else if (cursorPos == 2) kd = constrain(kd + rot * 0.5f, 0.0f, 200.0f);
    } else {
      cursorPos = (cursorPos + rot + N_ITEMS) % N_ITEMS;
    }
    banner = nullptr;
    drawMenu();
  }

  if (click) {
    banner = nullptr;
    switch (cursorPos) {
      case 0: case 1: case 2:
        adjusting = !adjusting;
        drawMenu();
        break;
      case 3:
        activeLevel = (activeLevel == HIGH) ? LOW : HIGH;
        saveSettings();
        drawMenu();
        break;
      case 4:
        adjusting = false;
        mode = MODE_SENSOR;
        break;
      case 5:
        motorTest();
        drawMenu();
        break;
      case 6:
        startRun();
        break;
    }
  }
}
