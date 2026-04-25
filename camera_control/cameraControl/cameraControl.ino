// ---------------------------------------------------------
// Automated Ball Tracking - MOTOR CONTROL (Final)
// ---------------------------------------------------------

const int stepPin = 9;         
const int dirPin = 5;          
const int enPin = 10;          
const int buttonPin1 = 12;    
const int buttonPin2 = 7;     

float angleX = 0, angleY = 0;
float normX = 0.0, normY = 0.0;

// Control Params (Updated from Python)
float Kp = 1.0;               
bool isTracking = false;      
float max_omega = 40.0;       // Speed limit from Python

// Camera geometry: how many pixels separate the optical axis from the edge
// of the frame horizontally. Python sends `normX` as a raw pixel delta in
// [-SENSOR_HALF_WIDTH_PX, +SENSOR_HALF_WIDTH_PX] (currently 320 px for the
// 640-wide capture stream). We normalise to [-1, +1] before applying the
// proportional gain so Kp keeps a physically meaningful unit:
//   Kp = 1.0  -> motor runs at exactly max_omega when the ball is at the edge
//   Kp = 0.5  -> half of max_omega at the edge (softer)
//   Kp > 1.0  -> saturates the clamp before the ball reaches the edge
// Update this constant if you change the camera capture width in Python.
const float SENSOR_HALF_WIDTH_PX = 320.0;

float omega = 0.0;            
float omega_preset = 0.0;     
const float manual_speed = 10.0; 

// Watchdog
uint32_t last_packet_time = 0;   
const uint32_t timeout_ms = 250; 

String inputBuffer = ""; 

void setup() {
  Serial.begin(115200);
  inputBuffer.reserve(64);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);  

  TCCR1A = 0; TCCR1B = 0; 
  TCCR1B |= (1 << WGM12);      
  TCCR1B |= (1 << CS11);       
  OCR1A = 11172; 
}

void loop() {
  // 1. SERIAL READ
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseIncomingData(inputBuffer);
      inputBuffer = ""; 
      last_packet_time = millis(); 
    } else {
      inputBuffer += inChar;
    }
  }

  // 2. STATUS
  bool python_active = (millis() - last_packet_time < timeout_ms);

  // 3. LOGIC
  bool btnLeft = (digitalRead(buttonPin1) == LOW);
  bool btnRight = (digitalRead(buttonPin2) == LOW);

  if (btnLeft) {
    omega_preset = -manual_speed;
  } 
  else if (btnRight) {
    omega_preset = manual_speed;
  } 
  else if (python_active && isTracking) {
    // AUTOMATIC MODE — Normalized P-control on pixel error.
    //
    //   error_fraction = normX / SENSOR_HALF_WIDTH_PX   (range [-1, +1])
    //   omega_preset   = error_fraction * max_omega * Kp
    //
    // This decouples three knobs that used to be tangled into a single
    // multiplication: the *physical* speed cap (`max_omega`, set in Python),
    // the *user gain* (`Kp`, set in Python), and the *sensor scale*
    // (`SENSOR_HALF_WIDTH_PX`, defined above). Result: Kp = 1.0 is always
    // "match max_omega at the screen edge", regardless of capture
    // resolution, regardless of the chosen speed cap.
    float error_fraction = normX / SENSOR_HALF_WIDTH_PX;
    omega_preset = error_fraction * max_omega * Kp;
    // Clamp matters only when Kp > 1.0 (intentional over-driving).
    if (omega_preset > max_omega) omega_preset = max_omega;
    if (omega_preset < -max_omega) omega_preset = -max_omega;
  } 
  else {
    omega_preset = 0;
    angleX = 0; 
  }

  // 4. SMOOTHING (ACCELERATION)
  static uint32_t tmr_accel;
  if (millis() - tmr_accel >= 1) {
    tmr_accel = millis();
    float accel_step = 0.15;
    if (abs(omega_preset - omega) > 0.05) {
      if (omega_preset > omega) omega += accel_step;
      else omega -= accel_step;
    } else {
      omega = omega_preset;
    }
  }

  // 5. MOTOR CONTROL
  if (abs(omega) > 0.1) {
    digitalWrite(enPin, LOW); 
    digitalWrite(dirPin, (omega > 0) ? HIGH : LOW);
    unsigned int new_ocr = 11172 / abs(omega);
    if (new_ocr < 20) new_ocr = 20; 
    OCR1A = new_ocr;
    TCCR1A |= (1 << COM1A0); 
  } 
  else {
    TCCR1A &= ~(1 << COM1A0);
    digitalWrite(stepPin, LOW);
  }
}

void parseIncomingData(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Search for 6 commas (7 values)
  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);
  int idx4 = line.indexOf(',', idx3 + 1);
  int idx5 = line.indexOf(',', idx4 + 1);
  int idx6 = line.indexOf(',', idx5 + 1);

  if (idx1 > 0 && idx6 > 0) {
    angleX     = line.substring(0, idx1).toFloat();
    angleY     = line.substring(idx1 + 1, idx2).toFloat();
    normX      = line.substring(idx2 + 1, idx3).toFloat();
    normY      = line.substring(idx3 + 1, idx4).toFloat();
    Kp         = line.substring(idx4 + 1, idx5).toFloat();
    isTracking = (line.substring(idx5 + 1, idx6).toInt() == 1);
    max_omega  = line.substring(idx6 + 1).toFloat();
  }
}