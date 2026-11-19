/*****************************************************************************
 * File: Zumo_Bluetooth_Border.ino
 *
 * Purpose:
 * Drive the Zumo with joystick commands received over Bluetooth (HC-05),
 * but when the border line is detected, override the joystick: back up a
 * little, turn around, then resume the latest joystick command.
 *
 * Commands:
 *   F, B, L, R = move
 *   S          = stop (joystick released)
 *   C          = toggle speed between 400 (fast) and 200 (slow)
 *
 * WIRING NOTE:
 * HC-05 TX -> pin 2, HC-05 RX -> pin 4 (same as the receiver sketch).
 * Border detection uses the four inner sensors (A3, 11, 6, 5), which do
 * not overlap with the Bluetooth pins.
 *****************************************************************************/

#include <Wire.h>
#include <ZumoShield.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

#define LED 13

/* ---- Tunables ---------------------------------------------------------- */
#define BORDER_MARGIN     500    // us; a sensor counts as 'border' when its reading
                                 // differs from its start-up baseline by more than this
#define QTR_TIMEOUT       2500   // us; max sensor read time

/* Joystick speed levels (toggled by 'C') */
#define SPEED_FAST        400
#define SPEED_SLOW        200

/* Debug switches: set to 0 once everything works */
#define RUN_STARTUP_TEST  1   // brief forward nudge at power-up (proves motors)
#define DEBUG_SENSORS     1   // print sensor readings twice a second
#define BORDER_ENABLED    1   // 0 = ignore sensors entirely (pure joystick)

/* Border override (fixed, not affected by the speed toggle) */
#define OVERRIDE_REVERSE_SPEED  200
#define OVERRIDE_TURN_SPEED     250
#define OVERRIDE_REVERSE_MS     250   // how far to back up
#define OVERRIDE_TURN_MS        600   // tune for ~180 degree turn
/* ------------------------------------------------------------------------ */

#define BT_RX_PIN  2    // HC-05 TX -> this pin (same as the working receiver sketch)
#define BT_TX_PIN  4    // HC-05 RX -> this pin

#define NUM_SENSORS 4
unsigned char sensorPins[NUM_SENSORS] = { A3, 11, 6, 5 };  // left -> right
unsigned int  sensorValues[NUM_SENSORS];
unsigned int  baseline[NUM_SENSORS];   // readings of the normal floor, taken at power-up

QTRSensorsRC qtr(sensorPins, NUM_SENSORS, QTR_TIMEOUT, QTR_NO_EMITTER_PIN);

SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);   // RX, TX
ZumoMotors motors;

/* Override state machine (non-blocking so Bluetooth bytes aren't missed) */
enum State { MANUAL, OVERRIDE_REVERSE, OVERRIDE_TURN };
State state = MANUAL;
unsigned long stateStart = 0;
int turnDir = 1;          // +1 = turn right, -1 = turn left
char lastCmd = 'S';       // latched joystick movement command
int  currentSpeed = SPEED_FAST;   // toggled by 'C'

void applyCommand(char cmd)
{
    switch (cmd)
    {
        case 'F': motors.setSpeeds(currentSpeed, currentSpeed);   break;
        case 'B': motors.setSpeeds(-currentSpeed, -currentSpeed); break;
        case 'L': motors.setSpeeds(-currentSpeed, currentSpeed);  break;
        case 'R': motors.setSpeeds(currentSpeed, -currentSpeed);  break;
        case 'S': motors.setSpeeds(0, 0);                         break;
        default:  break;
    }
}

bool isValidCommand(char c)
{
    return c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S' || c == 'C';
}

void toggleSpeed()
{
    currentSpeed = (currentSpeed == SPEED_FAST) ? SPEED_SLOW : SPEED_FAST;
    Serial.print("SPEED: ");
    Serial.println(currentSpeed);
}

bool sensorHit(int i)
{
    long diff = (long)sensorValues[i] - (long)baseline[i];
    return (diff > BORDER_MARGIN) || (diff < -BORDER_MARGIN);
}

void calibrateBaseline()
{
    /* Average several readings of the floor the bot is sitting on.
       Power the bot on with it placed inside the arena, NOT on the line. */
    unsigned long sum[NUM_SENSORS] = {0};
    const int samples = 10;
    for (int n = 0; n < samples; n++)
    {
        qtr.read(sensorValues);
        for (int i = 0; i < NUM_SENSORS; i++) sum[i] += sensorValues[i];
        delay(20);
    }
    Serial.print("BASELINE: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        baseline[i] = sum[i] / samples;
        Serial.print(baseline[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void startOverride(int dir)
{
    turnDir = dir;
    state = OVERRIDE_REVERSE;
    stateStart = millis();
    motors.setSpeeds(-OVERRIDE_REVERSE_SPEED, -OVERRIDE_REVERSE_SPEED);
    digitalWrite(LED, HIGH);
    Serial.println("BORDER! overriding");
}

void setup()
{
    // uncomment if necessary to correct motor directions
    //motors.flipLeftMotor(true);
    //motors.flipRightMotor(true);

    pinMode(LED, OUTPUT);
    Serial.begin(9600);
    bluetooth.begin(9600);

#if RUN_STARTUP_TEST
    motors.setSpeeds(SPEED_FAST, SPEED_FAST);
    delay(300);
#endif
    motors.setSpeeds(0, 0);
    delay(300);               // let the bot settle before reading the floor
    calibrateBaseline();
    Serial.println("Zumo Bluetooth + Border Ready");
}

void loop()
{
    /* 1. Always read Bluetooth. Movement commands are latched; 'C' toggles
          the speed. During an override they are stored but not acted on
          until the override ends. */
    while (bluetooth.available())
    {
        char cmd = bluetooth.read();
        if (!isValidCommand(cmd)) continue;

        Serial.print("RX: ");
        Serial.println(cmd);

        if (cmd == 'C')
        {
            toggleSpeed();
        }
        else
        {
            lastCmd = cmd;
        }

        /* Re-apply the latched command so a new command or a new speed
           takes effect immediately (not during an override). */
        if (state == MANUAL) applyCommand(lastCmd);
    }

    /* 2. Run the override state machine, or check for the border. */
    unsigned long now = millis();

    switch (state)
    {
        case MANUAL:
        {
#if !BORDER_ENABLED
            break;
#endif
            qtr.read(sensorValues);

#if DEBUG_SENSORS
            static unsigned long lastPrint = 0;
            if (now - lastPrint >= 500)
            {
                lastPrint = now;
                Serial.print("SENS: ");
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    Serial.print(sensorValues[i]);
                    Serial.print(' ');
                }
                Serial.print(" cmd=");
                Serial.println(lastCmd);
            }
#endif

            bool leftHit  = sensorHit(0) || sensorHit(1);
            bool rightHit = sensorHit(2) || sensorHit(3);

            /* Only react when moving toward the border (not when stopped or
               reversing), so you can still back away by hand. */
            bool movingForward = (lastCmd == 'F' || lastCmd == 'L' || lastCmd == 'R');

            if (movingForward && (leftHit || rightHit))
            {
                /* Line on the left -> turn right, otherwise turn left */
                startOverride(leftHit && !rightHit ? 1 : -1);
            }
            break;
        }

        case OVERRIDE_REVERSE:
            if (now - stateStart >= OVERRIDE_REVERSE_MS)
            {
                motors.setSpeeds(turnDir * OVERRIDE_TURN_SPEED,
                                 -turnDir * OVERRIDE_TURN_SPEED);
                state = OVERRIDE_TURN;
                stateStart = now;
            }
            break;

        case OVERRIDE_TURN:
            if (now - stateStart >= OVERRIDE_TURN_MS)
            {
                state = MANUAL;
                digitalWrite(LED, LOW);
                applyCommand(lastCmd);   // hand control back to the joystick
                Serial.println("Override done");
            }
            break;
    }
}
