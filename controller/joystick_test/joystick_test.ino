#define JS1_X 4   // Yaw
#define JS1_Y 3   // Throttle
#define JS1_SW 5

#define JS2_X 1   // Roll
#define JS2_Y 0   // Pitch
#define JS2_SW 2

int axis(int raw) {
  return -(raw - 2048);   // center + reverse
}

void setup() {
  Serial.begin(115200);

  pinMode(JS1_SW, INPUT_PULLUP);
  pinMode(JS2_SW, INPUT_PULLUP);
}

void loop() {
  // ---- Joystick 1 ----
  int yaw      = axis(analogRead(JS1_X));
  int throttle = axis(analogRead(JS1_Y));
  bool j1btn   = digitalRead(JS1_SW) == LOW;

  // ---- Joystick 2 ----
  int roll   = axis(analogRead(JS2_X));
  int pitch  = axis(analogRead(JS2_Y));
  bool j2btn = digitalRead(JS2_SW) == LOW;

  // ---- Output ----
  Serial.print("THR: "); Serial.print(throttle);
  Serial.print("  YAW: "); Serial.print(yaw);
  Serial.print(" | ROLL: "); Serial.print(roll);
  Serial.print("  PIT: "); Serial.print(pitch);

  Serial.print(" | B1:");
  Serial.print(j1btn ? "P" : "-");
  Serial.print(" B2:");
  Serial.println(j2btn ? "P" : "-");

  delay(100);
}
