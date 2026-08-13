/*
 * Automatic_Watering_PID.ino
 *
 * Automatic Plant Watering System
 * Arduino Uno + Soil Moisture Sensor + Relay + DC Motor/Pump
 *
 * PID Parameters:
 * Kp = 5.0
 * Ki = 0.8
 * Kd = 0.1
 *
 * Primary simulation condition:
 * Initial moisture ≈ 30%
 * Setpoint = 60%
 */

const int SOIL_MOISTURE_PIN = A0;
const int RELAY_PIN = 7;
const int LED_PIN = 13;

// PID parameters
const double Kp = 5.0;
const double Ki = 0.8;
const double Kd = 0.1;

// Target soil moisture
const double SETPOINT = 60.0;

// Timing
const unsigned long SAMPLE_TIME = 100;  // ms

// Soil moisture calibration for simulation
// Adjust according to the actual Tinkercad sensor configuration
const int SENSOR_DRY = 1023;
const int SENSOR_WET = 0;

// PID variables
double moisture = 0.0;
double error = 0.0;
double previousError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double pidOutput = 0.0;

unsigned long previousTime = 0;

void setup()
{
    Serial.begin(9600);

    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    previousTime = millis();

    Serial.println("========================================");
    Serial.println(" AUTOMATIC PLANT WATERING SYSTEM");
    Serial.println(" PID CONTROL");
    Serial.println("========================================");
    Serial.print("Kp = ");
    Serial.println(Kp);
    Serial.print("Ki = ");
    Serial.println(Ki);
    Serial.print("Kd = ");
    Serial.println(Kd);
    Serial.print("Setpoint = ");
    Serial.print(SETPOINT);
    Serial.println(" %");
    Serial.println("----------------------------------------");
}

void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= SAMPLE_TIME)
    {
        double dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        // Read soil moisture sensor
        int rawValue = analogRead(SOIL_MOISTURE_PIN);

        // Convert analog reading to soil moisture percentage
        moisture = map(rawValue, SENSOR_DRY, SENSOR_WET, 0, 100);

        // Limit moisture to 0-100%
        moisture = constrain(moisture, 0.0, 100.0);

        // Calculate PID error
        error = SETPOINT - moisture;

        // Integral term
        integral += error * dt;

        // Anti-windup
        integral = constrain(integral, -100.0, 100.0);

        // Derivative term
        derivative = (error - previousError) / dt;

        // PID output
        pidOutput =
            (Kp * error) +
            (Ki * integral) +
            (Kd * derivative);

        // Limit PID output
        pidOutput = constrain(pidOutput, 0.0, 255.0);

        /*
         * Relay-based pump control.
         *
         * Because a relay is fundamentally an ON/OFF actuator,
         * the PID output is converted into a software duty-cycle.
         * The duty-cycle determines how long the pump remains ON
         * within a fixed control window.
         */
        controlPump(pidOutput);

        // Store previous error
        previousError = error;

        // Serial output
        Serial.print("Raw Sensor = ");
        Serial.print(rawValue);

        Serial.print(" | Moisture = ");
        Serial.print(moisture, 2);
        Serial.print(" %");

        Serial.print(" | Error = ");
        Serial.print(error, 2);

        Serial.print(" | PID = ");
        Serial.print(pidOutput, 2);

        Serial.print(" | Pump = ");
        Serial.println(pidOutput > 0 ? "ON" : "OFF");
    }
}

void controlPump(double controlSignal)
{
    /*
     * Fixed control window:
     * 2 seconds = 2000 ms
     *
     * PID output:
     * 0   -> 0% duty cycle
     * 255 -> 100% duty cycle
     */

    const unsigned long WINDOW_SIZE = 2000;

    static unsigned long windowStart = 0;

    unsigned long currentTime = millis();

    if (windowStart == 0)
    {
        windowStart = currentTime;
    }

    if (currentTime - windowStart >= WINDOW_SIZE)
    {
        windowStart = currentTime;
    }

    double dutyCycle = controlSignal / 255.0;

    unsigned long onTime =
        (unsigned long)(WINDOW_SIZE * dutyCycle);

    unsigned long elapsed =
        currentTime - windowStart;

    if (elapsed < onTime && moisture < SETPOINT)
    {
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
    }
}