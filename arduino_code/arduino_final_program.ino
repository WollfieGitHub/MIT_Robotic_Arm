
//========================================================================================================//
//                                                                                                        //
//                  BEGIN          BEGIN            BEGIN               BEGIN       BEGIN                 //
//                                                                                                        //
//========================================================================================================//
const int NB_SERVOS = 5;

int servo_indices[NB_SERVOS] = {5, 1, 2, 3, 4};
int servo_targets[NB_SERVOS] = {550, 150, 300, 200, 0};
int servo_currents[NB_SERVOS] = {servo_targets[0],
                                 servo_targets[1],
                                 servo_targets[2],
                                 servo_targets[3],
                                 servo_targets[4]};

const double step_size = 0.05;

#include <arduino.h>
#include <SoftwareSerial.h>

#define RX 8 // TO PLUG INTO VOLTAGE DIVISOR
#define TX 11

int nb_disapointments = 0;

SoftwareSerial espSerial(RX, TX); // RX,

#include <HCPCA9685.h>
#include "HCPCA9685.h"

#define  I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);

// CAR PINS AND VARIABLES
//IR sensor pins
#define leftPin 9 //left IR sensor
#define rightPin 10 //right IR sensor

#define NB_ARM_SERVOS 5
#define FRAME_VAL_LENGTH 5
#define SERVO_REFRESH_DELAY 7

#define END_STRING_CHAR '\n'
#define BEGIN_ESP_RECEIVE_CHAR '?'
#define BEGIN_ESP_SEND_CHAR '!'
#define MAX_ITEM_IN_BASKET_SLOT 2
//Bridge motors pins
int enA = 6;
int in1 = 2;
int in2 = 4;
int enB = 3;
int in3 = 5;
int in4 = 7;
int motorSpeed = 230;

int basket_storage[4] = {0, 0, 0, 0};

// STATES AND VARIABLES
enum states {
    IDLE_STATE, MOVING, STOP_DETECTED, RESTART,
        GRAB, ANIMATE_GRAB, FINISH_GRAB,
        DROP, SEND_DROP, ANIMATE_DROP, FINISH_DROP,
        RESET
};
enum states state = IDLE_STATE;

// Basket Storage Slots
enum BASKET_SLOT {
    BOT_LEFT=0, BOT_RIGHT, TOP_LEFT, TOP_RIGHT, NONE
};



const String s_forward = "forward";
const String s_reset = "reset";
const String s_grab = "grab_begin";
const String s_frame = "frame";
const String s_grab_finished = "grab_end";
const String s_drop_finished = "drop_end";
const String s_end = "finish";
const String s_restart = "restart";

int number_articles = 0;

int force_moving_delay = 1000; //delay in ms when car starts moving again

//====================================================//
//                                                    //
//                  ANIMATIONS                        //
//                                                    //
//====================================================//

int nb_time_grab = 1;

states anim_next_state;
bool last_anim_set = false;

unsigned long last_servo_millis = millis();
unsigned long current_millis = millis();

BASKET_SLOT current_dropping_slot;

int servo_anim_last[5] = {0, 0, 0, 0, 0};

//====================================================//
//                                                    //
//                  SERIAL BUFFER                     //
//                                                    //
//====================================================//

String current_serial_buffer;
char last_read_serial_char;

//====================================================//
//                                                    //
//                  FUNCTIONS                         //
//                                                    //
//====================================================//

void transition_to(states next_state);

void update_servos();
void turn_to(int servo_idx, int angle);

void start_animation(states next_state);
void clear_animation();

void reset();
void listen_message();
void arm_grab();
BASKET_SLOT arm_drop();
void finished_arm_drop(BASKET_SLOT dropSlot);
bool contains(const String& s, const String& search);
bool handle_valid_ESP_msg(const String& message);

void try_turn_left();
void setup_car();

void follow_line();
void try_turn_right();
void try_go_forward();
void start_car();
void stop_car();

void esp_send(const String& msg);

//====================================================//
//                                                    //
//                  SETUP                             //
//                                                    //
//====================================================//

int linear_actualization(int i);

bool is_same_as_last_frame(int curr_frame, int last_frame);

#define BAUD_RATE 115200

#define MAX_DELTA_PULSE 80

#define SERVO_THRESHOLD 4

void setup() {
    Serial.begin(BAUD_RATE);
    espSerial.begin(BAUD_RATE);

    setup_car();

    HCPCA9685.Init(SERVO_MODE);
    HCPCA9685.Sleep(false);
    for(int i = 0; i < NB_SERVOS; i++) {
        HCPCA9685.Servo(servo_indices[i], servo_currents[i]);
    }

    state = IDLE_STATE;

    Serial.println("Bobby Shopping's Cart fully initialized and ready to go.");
}

//====================================================//
//                                                    //
//                  LOOP                              //
//                                                    //
//====================================================//

void loop() {

    /**
     * State handling
     */
    current_millis = millis();
    // Update all servos relative to their target
    if (current_millis - last_servo_millis > SERVO_REFRESH_DELAY) {
        update_servos();
        last_servo_millis = millis();
    }

    switch (state) {
        case IDLE_STATE:
            listen_message();
            break;

        case MOVING:
            follow_line();
            listen_message();
            break;

        case STOP_DETECTED:
            esp_send("stopped");
            listen_message();
            delay(10);
            break;

        case RESTART:
            follow_line();
            break;

        case GRAB:
            arm_grab();
            transition_to(ANIMATE_GRAB);
            break;

        case ANIMATE_GRAB:
            listen_message();
            break;

        case FINISH_GRAB:
            transition_to(DROP);
            break;

        case DROP:
            current_dropping_slot = arm_drop();
            transition_to(SEND_DROP);
            break;

        case SEND_DROP:
            esp_send("drop@" + (int)current_dropping_slot);
            listen_message();
            break;

        case ANIMATE_DROP:
            listen_message();
            break;

        case FINISH_DROP:
            finished_arm_drop(current_dropping_slot);
            nb_time_grab--;
            if (nb_time_grab > 0) {
                transition_to(GRAB);
                Serial.println("Another item has to be grabbed from the same shelves, repeating...");
            } else {
                follow_line();
            }
            break;

        case RESET:
            reset();
            break;
    }
}

void transition_to(states next_state) {
    state = next_state;
    switch (next_state) {
        case IDLE_STATE:    Serial.println("Waiting for instructions...");  break;
        case MOVING:        Serial.println("Going forward !");              break;
        case STOP_DETECTED: Serial.println("Stop detected !");              break;
        case GRAB:          Serial.println("Grab process initialized.");    break;
        case FINISH_GRAB:   Serial.println("Grab process completed");       break;
        case DROP:          Serial.println("Drop process initialized");     break;
        case SEND_DROP:     Serial.println("Sending drop slot...");         break;
        case ANIMATE_DROP:  Serial.println("Drop process pending...");      break;
        case FINISH_DROP:   Serial.println("Drop process completed");       break;
        case RESET:         Serial.println("Protocol reset completed");     break;
        default: break;
    }
}

//====================================================//
//                                                    //
//                  BASKET MANAGEMENT                 //
//                                                    //
//====================================================//

/**
 * Increment by one the count of items in the basket's slot
 * @param slot The basket slot
 */
void drop_an_item_at(BASKET_SLOT slot) {
    basket_storage[(int)slot] += 1;
}

/**
 *
 * @param slot A basket slot
 * @return True if the basket is full at the given slot
 */
bool basket_is_full_at(BASKET_SLOT slot) {
    return basket_storage[(int)slot] >= MAX_ITEM_IN_BASKET_SLOT;
}

/**
 * @return the next dropping slot (free basket slot)
 */
BASKET_SLOT get_next_dropping_slot() {
    for (int i = 0; i < NONE; ++i) {
        if ( not basket_is_full_at((BASKET_SLOT)i) ) {
            return (BASKET_SLOT)i;
        }
    }
    return NONE;
}

//====================================================//
//                                                    //
//                  ESP COMMAND HANDLER               //
//                                                    //
//====================================================//

/**
 * Returns true if {@code s} contains the substring {@code search}
 * @param s The source string
 * @param search The substring (pattern)
 * @return True if search is in s
 */
bool contains(const String& s, const String& search) {
    return s.indexOf(search) >= 0;
}

void esp_send(const String& msg) {
    String to_send = BEGIN_ESP_SEND_CHAR + msg;
    espSerial.println(to_send);
    espSerial.flush();
}

/**
 * Serial string rebuilding method used because of the time sensitiveness of the program
 * Builds a character on each call so it doesn't block the program in a while loop
 * @return true if the string is fully built, false otherwise
 */
bool serial_msg_built() {
    while (espSerial.available()) {

        last_read_serial_char = (char) espSerial.read();

        if (current_serial_buffer.length() == 0 && last_read_serial_char != BEGIN_ESP_RECEIVE_CHAR) {
            // DO NOTHING

        } else {
            // We need to add more to the string
            current_serial_buffer += last_read_serial_char;

            if (last_read_serial_char == END_STRING_CHAR) {
                // The string is fully built
                return true;
            }
        }
    }
    return false;
}

void serialEvent() {
    /*if(handle_valid_ESP_msg(Serial.readString())) {
        Serial.println("Received and treated");
    }*/
    String input = Serial.readString();
    espSerial.println(input);
}

/**
 * Tries to build a new serial msg, or append a character if one is available (on each call, should be
 * called each loop iteration)
 * If a message was built, calls the {@code handle_valid_ESP_msg} procedure to handle it
 */
void listen_message() {
    if (serial_msg_built()) {
        if (handle_valid_ESP_msg(current_serial_buffer)) {
            esp_send("received");
        }
        current_serial_buffer = "";
        last_read_serial_char = '\0';
    }
}

/**
 * Tries matching the given message with every valid ESP pattern and acts in consequence
 * @param message The message from the ESP
 * @return True if it was a valid ESP message and it has been processed
 */
bool handle_valid_ESP_msg(const String& message) {
    Serial.print("Received a new message \" ");
    Serial.print(message);
    Serial.print("\" ");

    if (contains(message, s_reset)) {
        Serial.println(" -> RESET command received.");
        transition_to(RESET);
        return true;
    } else if (contains(message, s_forward)) {
        Serial.println(" -> MOVING command received.");
        transition_to(MOVING);
        return true;

    } else if (contains(message, s_restart)) {
        Serial.println(" -> RESTART command received");
        transition_to(RESTART);
        return true;

    } else if (contains(message, s_grab)) {
        // Parse the nb of items it has to grab from the command
        String grab_msg_spit = message.substring(s_grab.length()+1,s_grab.length()+2);
        nb_time_grab = (int)grab_msg_spit.toInt();
        String msg = "\"" + message + "\" : \"" + grab_msg_spit + "\"";
        Serial.println(msg);
        Serial.print(" -> GRAB command received, ");
        // Check for a valid
        if (0 < nb_time_grab && nb_time_grab < 4 ) {
            Serial.print(nb_time_grab);
            Serial.println(" items to grab.");
            transition_to(GRAB);
            return true;
        } else {
            Serial.println("INVALID NUMBER");
            return false;
        }

    } else if (contains(message, s_grab_finished) && state == ANIMATE_GRAB) {
        Serial.println(" -> GRAB_FINISHED command received");
        clear_animation();
        return true;

    } else if (contains(message, s_drop_finished)) {
        Serial.println(" -> DROP_FINISHED command received");
        clear_animation();
        return true;

    } else if (contains(message, s_frame)){
        Serial.println(" -> FRAME command received.");
        String frame = message.substring(s_frame.length()+1, message.length());
        // Received a frame
        String frame_s;
        String debug_msg;

        if (state == SEND_DROP) {
            transition_to(ANIMATE_DROP);
        }

        bool valid = true;
        for (int i = 0; i < NB_ARM_SERVOS; ++i) {
            Serial.print("-> {");
            Serial.print(i);
            Serial.print("} ");
            frame_s =  frame.substring(i * FRAME_VAL_LENGTH, (i + 1) * FRAME_VAL_LENGTH);
            String val_msg;
            Serial.print("substringaide");

            bool is_valid_number = true;
            for (int j = 0; j < FRAME_VAL_LENGTH; ++j) {
                if (is_valid_number && not isDigit(frame_s[j])) {
                    is_valid_number = false;
                }
            }

            Serial.print(" - valid_number determined");

            int frame_value;
            if (is_valid_number) {
                frame_value = (int)frame_s.toInt();
                if (i == 4 && frame_value >= 70) {
                    frame_value = 70;
                }
            } else {
                frame_value = -1;
            }

            Serial.print(" - converted frame to int");

            if (last_anim_set && is_same_as_last_frame(frame_value,servo_anim_last[i]) ) {

                turn_to(i, frame_value);
                val_msg = frame_value;
            } else {
                nb_disapointments++;
                val_msg = "INVALID : ";
                val_msg += frame_value;
                valid = false;
            }

            debug_msg = "\t - Servo {";
            debug_msg += (String)i;
            debug_msg += "}, last was ";
            debug_msg += servo_anim_last[i];
            debug_msg += ", now received order ";
            debug_msg += val_msg;
            Serial.println(debug_msg);
            servo_anim_last[i] = frame_value;
        }

        Serial.print("Nb disapointments = ");
        Serial.println(nb_disapointments);

        if (!last_anim_set) {
            last_anim_set = true;
        }
        return valid;

    } else if (contains(message, s_end)) {
        Serial.println(" -> END");
        transition_to(RESET);
        return true;

    }
    Serial.println(". Was INVALID : " + message);
    return false;
}

bool is_same_as_last_frame(int curr_frame, int last_frame) {
    return curr_frame != -1 && last_frame != -1 && curr_frame == last_frame;
}

//====================================================//
//                                                    //
//                  ARM FUNCTION                      //
//                                                    //
//====================================================//

/**
 * Sets the given servo's target to the given angle/pulsation
 * @param servo_idx The index of the servo (0 base to 4 clamp)
 * @param angle The target angle/pulsation
 */
void turn_to(int servo_idx, int angle) {
    servo_targets[servo_idx] = angle;
}

/**
 * Records the state to transition to when the animation is done and set is_animating to true
 * @param next_state The state to transition to
 */
void start_animation(states next_state) {
    anim_next_state = next_state;
    last_anim_set = true;
}

/**
 * Set is_animating to false and transition to the next state
 */
void clear_animation() {
    transition_to(anim_next_state);
    last_anim_set = false;
}

/**
 * Updates all servos of the arm to be 5% closer to their target
 */
void update_servos() {
    for(int i = 0; i < NB_SERVOS; i++) {
        int servo_step = (int)(step_size * servo_targets[i] + (1 - step_size) * servo_currents[i]);
        //int servo_step = linear_actualization(i);
        servo_currents[i] = servo_step;
        HCPCA9685.Servo(servo_indices[i], servo_step);
    }
}

int linear_actualization(int i) {
    int servo_step;
    int servo_step_dir = 1;
    if (abs(servo_targets[i] - servo_currents[i]) < SERVO_THRESHOLD) {
        servo_step = servo_targets[i];
    } else {
        if (servo_targets[i] < servo_currents[i]) {
            servo_step_dir = -1;
        }
        servo_step = servo_currents[i] + (servo_step_dir) * SERVO_THRESHOLD;
    }
    return servo_step;
}

//====================================================//
//                                                    //
//                  ARM FUNCTION                      //
//                                                    //
//====================================================//

/**
 * Starts the animation with a future transition to FINISH_GRAB on finish
 */
void arm_grab() {
    /**
     * GRAB THE ITEM
     */
    start_animation(FINISH_GRAB);
}

/**
 * Starts the animation with a future transition to FINISH_DROP and return next dropping slot for the drop
 * @return The next dropping slot (free) or NONE if none available
 */
BASKET_SLOT arm_drop() {
    /**
     * DROP THE ITEM
     */

    start_animation(FINISH_DROP);
    return get_next_dropping_slot();
}

/**
 * Updates the basket's state (add an item into the given slot) and increment the number of articles
 * @param dropSlot The slots of the basket the item was dropped in
 */
void finished_arm_drop(BASKET_SLOT dropSlot) {
    /**
     * UPDATE PROGRAM STATE
     */
    drop_an_item_at(dropSlot);
    number_articles += 1;
}

//====================================================//
//                                                    //
//                  CAR FUNCTION                      //
//                                                    //
//====================================================//

/**
 * The car moves along the black line, and stops at some stops to send a scanning message to the ESP32
 */
void follow_line() {

    int leftValue = digitalRead(leftPin); // Read value from left sensor
    int rightValue = digitalRead(rightPin); // Read value from right sensor
    bool need_to_stop = leftValue == HIGH && rightValue == HIGH;

    if (need_to_stop && state == MOVING) {
        stop_car();
        transition_to(STOP_DETECTED);
    } else {
        /**
         * Restart the car after it stopped on a line here
         */
        if (state != MOVING) {
            transition_to(MOVING);
            start_car();
            delay(force_moving_delay);
        }
        try_turn_left();
        try_turn_right();
        try_go_forward();
    }
}

/**
 * If lightness detectors sense the car is to the right of the black line,
 * turn left until it is following the line
 */
void try_turn_left() {
    int leftValue = digitalRead(leftPin);
    int rightValue = digitalRead(rightPin);

    while (leftValue == HIGH && rightValue == LOW) {
        digitalWrite(in1, HIGH );
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW );
        digitalWrite(in4, HIGH);

        analogWrite(enA, motorSpeed);
        analogWrite(enB, motorSpeed);

        leftValue = digitalRead(leftPin);
        rightValue = digitalRead(rightPin);
    }
}

/**
 * If lightness detectors sense the car is to the left of the black line,
 * turn right until it is following the line
 */
void try_turn_right() {
    int leftValue = digitalRead(leftPin);
    int rightValue = digitalRead(rightPin);

    while (leftValue == LOW && rightValue == HIGH) {
        digitalWrite(in1, LOW );
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH );
        digitalWrite(in4, LOW);

        analogWrite(enA, motorSpeed);
        analogWrite(enB, motorSpeed);

        leftValue = digitalRead(leftPin);
        rightValue = digitalRead(rightPin);
    }
}

/**
 * If the lightness detectors sense the car is following the line, go forward
 */
void try_go_forward() {
    int leftValue = digitalRead(leftPin);
    int rightValue = digitalRead(rightPin);

    while (leftValue == LOW && rightValue == LOW) {
        digitalWrite(in1, HIGH );
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH );
        digitalWrite(in4, LOW);

        analogWrite(enA, motorSpeed);
        analogWrite(enB, motorSpeed);

        leftValue = digitalRead(leftPin);
        rightValue = digitalRead(rightPin);
    }
}

/**
 * Set all pinModes for the car's motors and stops the car
 */
void setup_car()
{
    //IR sensors setup
    pinMode(leftPin, INPUT);
    pinMode(rightPin, INPUT);

    //Bridge motors setup
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

/**
 * Writes the speed and pin so that the car can move forward
 */
void start_car() {
    digitalWrite(in1, HIGH );
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH );
    digitalWrite(in4, LOW);

    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

/**
 * Stops the car
 */
void stop_car() {
    digitalWrite(in1, LOW );
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW );
    digitalWrite(in4, LOW);

    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

/**
 * Reset for a new journey
 */
void reset() {
    state = IDLE_STATE;
    stop_car();
    number_articles = 0;
}