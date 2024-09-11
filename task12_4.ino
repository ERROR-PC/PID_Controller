// PID Class for General Use
class PID {
  private:
    float kp, ki, kd;
    float prevError, integral;

  public:
    PID(float Kp, float Ki, float Kd) {
      kp = Kp;
      ki = Ki;
      kd = Kd;
      prevError = 0;
      integral = 0;
    }

    float compute(float setpoint, float measuredValue, float deltaTime) {
      float error = setpoint - measuredValue;
      integral += error * deltaTime;
      float derivative = (error - prevError) / deltaTime;
      float output = kp * error + ki * integral + kd * derivative;
      prevError = error;
      return output;
    }
};

// Motor control setup (as an example)
const int motorPin = 9; // PWM output pin to motor
int speedSensorPin = A0; // Speed sensor connected to analog input
PID motorPID(2.0, 0.5, 0.1); // Tuning parameters for the PID

void setup() {
  pinMode(motorPin, OUTPUT);
}

float softStart(float previousValue, float newValue, float alpha) {
  return (alpha * newValue) + ((1 - alpha) * previousValue);
}

void loop() {
  float dt = 0.1; // a reading every 100ms
  float setpoint = 100;  // Desired speed
  float measuredSpeed = analogRead(speedSensorPin);  // reading actual motor speed
  float pidOutput = motorPID.compute(setpoint, measuredSpeed, dt);

  // Apply the soft start filter
  // static prevents the value from previous loop from
  // being lost
  static float smoothedOutput = 0;
  float alpha = 0.1;  // Smoothing factor (lower values make it smoother)
  smoothedOutput = softStart(smoothedOutput, pidOutput, alpha);
  
  // analogRead range: 0 - 1023
  // analogWrite range: 0 - 255
  // Constrain and map PID output to PWM range (0-255)
  int mappedOutput = map(smoothedOutput, 0, 1023, 0, 255);
  int motorPWM = constrain(mappedOutput, 0, 255);
  analogWrite(motorPin, motorPWM);

  delay(100); // 100ms delay
}
