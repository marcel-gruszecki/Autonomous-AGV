#include <Arduino.h>
const int PWM_FREQ = 200, PWM_BITS = 8;
#define PWM_SETUP(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
#define PWM_WRITE(pin, ch, duty) ledcWrite(ch, duty)

class AVG {
    private:
        const int IN1 = 23, IN2 = 22, IN3 = 19, IN4 = 18, ENA = 17, ENB = 16;
        const int SENS_L = 32, SENS_M = 35, SENS_R = 34;

        const bool BLACK_IS_HIGH = true;   
        const int   BASE_SPEED = 110;      
        const float KP = 35.0f;            
        const float KD = 60.0f;    
     
        const int   MIN_WHEEL = 70;        
        const int   SEARCH_KICK = 110;                // short kick so both wheels start moving
        const int   SEARCH_SPEED = 70;                // rotation speed while searching for the line
        const unsigned long SEARCH_MS = 600;          // how long to search in the first direction
        const unsigned long LOST_MS = 300;            // this many ms of white only = line lost

        float lastErr = 0;

        void motor(int inA, int inB, int en, int ch, int speed) {
            digitalWrite(inA, speed > 0 ? HIGH : LOW);
            digitalWrite(inB, speed < 0 ? HIGH : LOW);
            PWM_WRITE(en, ch, abs(speed));
        }
    
    public: 
        AVG() {
            pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
            pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
            PWM_SETUP(ENA, 0);
            PWM_SETUP(ENB, 1);
            pinMode(SENS_L, INPUT); pinMode(SENS_M, INPUT); pinMode(SENS_R, INPUT);
            drive(0, 0);
        }

        void drive(int left, int right) {
            motor(IN3, IN4, ENB, 1, left);
            motor(IN1, IN2, ENA, 0, right);
        }

        bool onLine(int pin) {
            return digitalRead(pin) == (BLACK_IS_HIGH ? HIGH : LOW);
        }

        // Rotates in place towards dir (-1 left, +1 right) until the middle sensor
        // sees the line or timeout_ms passes. Returns true if the line was found.
        bool find_line(int dir, unsigned long timeout_ms) {
            drive(dir * SEARCH_KICK, -dir * SEARCH_KICK);
            delay(20);
            drive(dir * SEARCH_SPEED, -dir * SEARCH_SPEED);
            unsigned long t0 = millis();
            while (!onLine(SENS_M)) {
                if (millis() - t0 >= timeout_ms) { drive(0, 0); return false; }
            }
            drive(0, 0);
            return true;
        }

        // First towards dir, then the other way (twice as long, it passes the start again).
        bool find_line_both(int dir) {
            return find_line(dir, SEARCH_MS) || find_line(-dir, 2 * SEARCH_MS);
        }

        // lines = number of cross lines to drive over; earlier intersections are
        // passed without stopping, only the last line ends the drive
        void follow_line(bool stop_after_line, int lines = 1) {
            const unsigned long AFTER_LINE_MS = 275;
            unsigned long crossed_at = 0;
            unsigned long lost_since = 0;
            int last_side = 0;    // +1 = line last seen by the right sensor, -1 = left
            bool armed = false;   // a cross line counts only after leaving the previous one
            lastErr = 0;

            while (true) {
                bool l = onLine(SENS_L), c = onLine(SENS_M), r = onLine(SENS_R);
                bool cross = l && c && r;

                if (!cross) armed = true;

                if (armed && cross && crossed_at == 0) {
                    if (--lines > 0) armed = false;   // intersection on the way: keep going straight
                    else if (!stop_after_line) break; // FORWARD: stop on the stop bar
                    else crossed_at = millis();       // FORWARD_AFTER_LINE: drive on a bit
                }
                if (crossed_at != 0 && millis() - crossed_at >= AFTER_LINE_MS) break;

                if (l != r) last_side = r ? 1 : -1;

                // all sensors off the line -> short search, first on the side
                // where the line was seen last
                if (!l && !c && !r) {
                    if (lost_since == 0) lost_since = millis();
                    if (millis() - lost_since >= LOST_MS) {
                        if (!find_line_both(last_side >= 0 ? 1 : -1)) break;
                        lost_since = 0;
                        lastErr = 0;
                        continue;
                    }
                } else {
                    lost_since = 0;
                }

                float err = (r ? 1.0f : 0.0f) - (l ? 1.0f : 0.0f);
                if (c && (l != r)) err *= 0.5f;

                float corr = KP * err + KD * (err - lastErr);
                lastErr = err;

                drive(constrain((int)(BASE_SPEED + corr), MIN_WHEEL, 255),
                    constrain((int)(BASE_SPEED - corr), MIN_WHEEL, 255));
                delay(2);
            }
            drive(-150, -150);
            delay(30);
            drive(0, 0);
        }

        void turn_left() {
            bool SENS_M_PASS = false;
            drive(-110, 110);
            delay(20);
            drive(-80, 80);
            delay(100);
            while (!onLine(SENS_L)) {
                if(onLine(SENS_M)) { SENS_M_PASS = true; }
            }
            if(onLine(SENS_M)) { SENS_M_PASS = true; }
            drive(110, -110);
            delay(30);
            drive(0, 0);
        }

        void turn_right() {
            bool SENS_M_PASS = false;
            drive(110, -110);
            delay(20);
            drive(80, -80);
            delay(100);
            while (!onLine(SENS_R)) {
                if(onLine(SENS_M)) { SENS_M_PASS = true; }
            }
            if(onLine(SENS_M)) { SENS_M_PASS = true; }
            drive(-150, 150);
            delay(30);
            drive(0, 0);
        }

        // 180 degree turn over the left side
        void return_back_left() {
            bool SENS_M_PASS = false;
            drive(-110, 110);
            delay(250);
            drive(-60, 60); // 80 for robot 2, 60 for the others
            delay(100);
            while (!onLine(SENS_L)) {
                if(onLine(SENS_M)) { SENS_M_PASS = true; }
            }
            if(onLine(SENS_M)) { SENS_M_PASS = true; }
            drive(150, -150);
            delay(80);
            drive(0, 0);
        }

        // 180 degree turn over the right side (mirror of return_back_left)
        void return_back_right() {
            bool SENS_M_PASS = false;
            drive(110, -110);
            delay(250);
            drive(60, -60); // 80 for robot 2, 60 for the others
            delay(100);
            while (!onLine(SENS_R)) {
                if(onLine(SENS_M)) { SENS_M_PASS = true; }
            }
            if(onLine(SENS_M)) { SENS_M_PASS = true; }
            drive(-150, 150);
            delay(80);
            drive(0, 0);
        }


    };

