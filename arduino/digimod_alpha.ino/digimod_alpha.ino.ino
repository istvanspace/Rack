#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

// Constants
#define MAX_BUFFER_SIZE 256
#define MAX_PAYLOAD_SIZE 20
#define END_MARKER 0xFF
#define START_MARKER 0xFE

// Module Settings
const char* LIBRARY_NAME = "Fundamental";
const char* MODULE_SLUG = "VCO";

// PIN SETTINGS
const int BUTTON_1_PIN = 2;
const int BUTTON_2_PIN = 3;
const int RED_LED_PIN = 8;
const int GREEN_LED_PIN = 9;
const int POT_PINS[] = {
  A0,
  A1, 
};

const int numPotPins = sizeof(POT_PINS) / sizeof(POT_PINS[0]);

// Array of pins to be checked for jack connection
const int jackPinsConnected[] = {
    4,
    5,
};

const int numJackPins = sizeof(jackPinsConnected) / sizeof(jackPinsConnected[0]);

// LED SETTINGS
const int slow_blink = 250;
const int fast_blink = 75;

// PROTOCOL SETTINGS
const uint8_t COMMAND_TYPE_INITIALIZE = 0x01;
const uint8_t COMMAND_TYPE_PARAMETER = 0x02;
const uint8_t COMMAND_TYPE_CONNECTED = 0x03;
const uint8_t COMMAND_TYPE_DISCONNECTED = 0x04;
const uint8_t COMMAND_TYPE_MATCHED = 0x05;
const uint8_t COMMAND_TYPE_ACKNOWLEDGEMENT = 0xFB;
const uint8_t COMMAND_RESULT_SUCCESS = 0x01;
const uint8_t COMMAND_RESULT_ERROR = 0x00;

struct Command {
    uint8_t commandId;
    uint8_t commandType;
    const char* payload1;
    const char* payload2;
    bool acknowledged;
};

// JACK CONNECTOR SETTINGS
struct PinStates {
  uint8_t portB;
  uint8_t portC;
  uint8_t portD;
  uint8_t portE;
  uint8_t portF;
  uint8_t portG;
  uint8_t portH;
  uint8_t portJ;
  uint8_t portK;
  uint8_t portL;
};

// Global variables
bool g_initialized = false;
bool waiting_for_match = false;
int g_lastPotValue[sizeof(POT_PINS) / sizeof(POT_PINS[0])] = { -1 };
Command g_currentCommand;
PinStates previousStates;

// Global variables for LED blinking
struct BlinkInfo {
    int pin;
    int count;
    int pause;
    unsigned long lastToggle;
    bool ledState;
    int toggleCount;
    bool initialState;  // Added to track the initial state
};
BlinkInfo blinkInfos[2]; 


// Function prototypes
void setupPins();
void setupModule();
void setupBlinkLED();
void handlePotentiometer();
void blinkLED(int pin, int count, int pause);
void updateLEDBlink();
void sendMessage(const Command& cmd);
void processIncomingMessage();
void processResponse(const uint8_t* buffer, size_t size, Command& lastCommand);
void moduleInitialized();
PinStates readAllPins();
void checkPinsWithFrequency(unsigned long interval);
void processPinStateChange(uint8_t previousState, uint8_t currentState, uint8_t pinMask, const char* portName, uint8_t pinNumber);
void updateLEDStatus();

void setup() {
    activateAllPullUps();
    setupPins();
    updateLEDStatus();
    Serial.begin(230400);
    setupModule();
    setupBlinkLED();
    previousStates = readAllPins(); // Initialize previous pin states
}

void loop() {
    if (g_initialized) {
        handlePotentiometer();
        checkPinsWithFrequency(100); // Check every 100 milliseconds (.1 second)
    } 
    if (!g_currentCommand.acknowledged){
        processIncomingMessage();
    }
    updateLEDBlink();
}

void setupPins() {
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    for (int i = 0; i < numPotPins; i++) {
      pinMode(POT_PINS[i], INPUT);
    }

    // Set up the jack pins with internal pull-up resistors
    for (int i = 0; i < numJackPins; i++) {
        pinMode(jackPinsConnected[i], INPUT_PULLUP);
    }
}

void setupBlinkLED() {
    blinkInfos[0] = {RED_LED_PIN, 0, 0, 0, false, 0, false};
    blinkInfos[1] = {GREEN_LED_PIN, 0, 0, 0, false, 0, false};
}

void setupModule() {
    Command initializeCommand = {15, COMMAND_TYPE_INITIALIZE, LIBRARY_NAME, MODULE_SLUG, false};
    g_currentCommand = initializeCommand;
    sendMessage(g_currentCommand);
}

inline int intAbs(int value) {
    return (value < 0) ? -value : value;
}

void handlePotentiometer() {
    static Command potCommand = {2, COMMAND_TYPE_PARAMETER, "", "", true};
    static char potValueStr[6];
    static char pinIndexStr[4];

    for (int i = 0; i < numPotPins; i++) {
        int potValue = analogRead(POT_PINS[i]);
        if (intAbs(potValue - g_lastPotValue[i]) > 2) {
            g_lastPotValue[i] = potValue;
            snprintf(potValueStr, sizeof(potValueStr), "%d", potValue);
            snprintf(pinIndexStr, sizeof(pinIndexStr), "%d", i);

            potCommand.payload1 = pinIndexStr;
            potCommand.payload2 = potValueStr;

            g_currentCommand = potCommand;
            sendMessage(potCommand);
            blinkLED(RED_LED_PIN, 1, fast_blink);
        }
    }
}

//void handlePotentiometer() {
//    int potValue = analogRead(POT_PIN);
//    if (abs(potValue - g_lastPotValue) > 2) {
//        g_lastPotValue = potValue;
//        char potValueStr[6];
//        snprintf(potValueStr, sizeof(potValueStr), "%d", potValue);
//        
//        Command potCommand = {2, COMMAND_TYPE_PARAMETER, "0", potValueStr, true};
//        g_currentCommand = potCommand;
//        sendMessage(potCommand);
//        blinkLED(RED_LED_PIN, 1, fast_blink);
//    }
//}

void sendMessage(const Command& cmd) {
    uint8_t buffer[MAX_BUFFER_SIZE];
    uint8_t* ptr = buffer;
    
    *ptr++ = START_MARKER;
    
    uint8_t payload1Length = strlen(cmd.payload1);
    uint8_t payload2Length = strlen(cmd.payload2);
    uint16_t messageLength = 4 + payload1Length + payload2Length;
    
    *ptr++ = (messageLength >> 8) & 0xFF;
    *ptr++ = messageLength & 0xFF;
    *ptr++ = cmd.commandId;
    *ptr++ = cmd.commandType;
    *ptr++ = payload1Length;
    *ptr++ = payload2Length;
    
    memcpy(ptr, cmd.payload1, payload1Length);
    ptr += payload1Length;
    memcpy(ptr, cmd.payload2, payload2Length);
    ptr += payload2Length;
    
    *ptr++ = END_MARKER;
    
    Serial.write(buffer, ptr - buffer);
    Serial.flush();
}

void processIncomingMessage() {
    static size_t bufferIndex = 0;
    static uint8_t buffer[MAX_BUFFER_SIZE];
    static bool messageStarted = false;
    static uint16_t expectedLength = 0;
    
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        if (!messageStarted && byte == START_MARKER) {
            messageStarted = true;
            bufferIndex = 0;
            continue;
        }
        
        if (messageStarted) {
            buffer[bufferIndex++] = byte;
            
            if (bufferIndex == 2) {
                expectedLength = (buffer[0] << 8) | buffer[1];
                continue;
            }
            
            if (bufferIndex == expectedLength + 3) {
                if (byte == END_MARKER) {
                    blinkLED(GREEN_LED_PIN, 2, fast_blink);
                    processResponse(buffer + 2, bufferIndex - 3, g_currentCommand);
                } else {
                    blinkLED(RED_LED_PIN, 3, fast_blink);
                }
                
                messageStarted = false;
                bufferIndex = 0;
                break;
            }
        }
        
        if (bufferIndex >= MAX_BUFFER_SIZE) {
            blinkLED(RED_LED_PIN, 4, slow_blink);
            messageStarted = false;
            bufferIndex = 0;
        }
    }
}

void processResponse(const uint8_t* buffer, size_t size, Command& lastCommand) {
    if (size < 4) { // Command ID + Type + 2 payload lengths
        // message size error
        blinkLED(RED_LED_PIN, 4, slow_blink);
        return;
    }
    
    uint8_t commandId = buffer[0];
    uint8_t commandType = buffer[1];
    uint8_t payload1Length = buffer[2];
    uint8_t payload2Length = buffer[3];
    
    if (size < 4 + payload1Length + payload2Length) {
        // message size error
        blinkLED(RED_LED_PIN, 4, slow_blink);
        return;
    }

    char payload1[MAX_PAYLOAD_SIZE] = {0};
    memcpy(payload1, buffer + 4, payload1Length);
    
    char payload2[MAX_PAYLOAD_SIZE] = {0};
    memcpy(payload2, buffer + 4 + payload1Length, payload2Length);

    if (commandType == COMMAND_TYPE_ACKNOWLEDGEMENT) {
//        uint8_t payload1 = buffer[4];
        
        if (lastCommand.commandId == commandId) {
            
            if (static_cast<uint8_t>(payload1[0]) == COMMAND_RESULT_SUCCESS) {
//              blinkLED(RED_LED_PIN, 2, slow_blink);
              lastCommand.acknowledged = true;
              
              if (lastCommand.commandType == COMMAND_TYPE_INITIALIZE) {
                  moduleInitialized();
              } else if (lastCommand.commandType == COMMAND_TYPE_CONNECTED) {
                  if (!waiting_for_match) {
                    waiting_for_match = true;
                    blinkLED(GREEN_LED_PIN, -1, slow_blink);
                  } else {
                    waiting_for_match = false;
                    stopBlinkLED(GREEN_LED_PIN);
                  }
              } else if (lastCommand.commandType == COMMAND_TYPE_DISCONNECTED) {
                  if (waiting_for_match) {
                    waiting_for_match = false;
                    stopBlinkLED(GREEN_LED_PIN);
                  } else {
                    waiting_for_match = true;
                    blinkLED(GREEN_LED_PIN, -1, slow_blink);
                  }
              } 
            } else if (static_cast<uint8_t>(payload1[0]) == COMMAND_RESULT_ERROR) {
              blinkLED(RED_LED_PIN, 3, slow_blink);
              sendMessage(lastCommand);
            } else {
              // unkown command result
              blinkLED(RED_LED_PIN, 4, slow_blink);
              blinkLED(GREEN_LED_PIN, 4, slow_blink);
              return;                       
            }
        } else {
          // unkown commandId
          blinkLED(RED_LED_PIN, 4, slow_blink);
          return;
        }
    } else if (commandType == COMMAND_TYPE_MATCHED) {
      
      if (waiting_for_match || lastCommand.commandType == COMMAND_TYPE_CONNECTED) {
        waiting_for_match = false;
        stopBlinkLED(GREEN_LED_PIN);
      }
      
    } else {
      // unknown command type
      blinkLED(RED_LED_PIN, 4, slow_blink);
      return;
    }
    
}

void moduleInitialized() {
  g_initialized = true;
  updateLEDStatus();
}

void updateLEDStatus() {
    digitalWrite(RED_LED_PIN, !g_initialized);
    digitalWrite(GREEN_LED_PIN, g_initialized);
    blinkInfos[0] = {RED_LED_PIN, 0, 0, 0, false, 0, !g_initialized};
    blinkInfos[1] = {GREEN_LED_PIN, 0, 0, 0, false, 0, g_initialized};
}

// Function to read all digital pins using direct port manipulation
PinStates readAllPins() {
  PinStates states;

  // Directly read the state of each port
  states.portB = PINB;
  states.portC = PINC;
  states.portD = PIND;
  states.portE = PINE;
  states.portF = PINF;
  states.portG = PING;
  states.portH = PINH;
  states.portJ = PINJ;
  states.portK = PINK;
  states.portL = PINL;

  return states;
}

// Function to activate pull-ups on all pins
void activateAllPullUps() {
    for (int j = 0; j < numJackPins; j++) {
            int pinNumber = jackPinsConnected[j];
            pinMode(pinNumber, INPUT_PULLUP);
    }
}

// Function to process the state change for a specific pin
void processPinStateChange(uint8_t previousState, uint8_t currentState, uint8_t pinMask, const char* portName, uint8_t pinNumber) {
    uint8_t prevPinState = previousState & pinMask;
    uint8_t currPinState = currentState & pinMask;

    if (prevPinState == 0 && currPinState != 0) {
        // Jack pulled out
        Command jackOutCommand = {2, COMMAND_TYPE_DISCONNECTED, pinNumber, 0, false};
        g_currentCommand = jackOutCommand;
        sendMessage(jackOutCommand);
        blinkLED(GREEN_LED_PIN, -1, fast_blink);
        
//        if (waiting_for_match) {
////          Serial.print("Jack pulled out before match from ");
////          Serial.print(portName);
////          Serial.print(" pin ");
////          Serial.println(pinNumber);
//          Command jackOutCommand = {2, COMMAND_TYPE_DISCONNECTED, pinNumber, 0, false};
//          g_currentCommand = jackOutCommand;
//          sendMessage(jackOutCommand);
//          blinkLED(GREEN_LED_PIN, -1, fast_blink);
//        } else {
////          Serial.print("Jack pulled out after match from ");
////          Serial.print(portName);
////          Serial.print(" pin ");
////          Serial.println(pinNumber);
//          blinkLED(GREEN_LED_PIN, -1, fast_blink);
//        }
        
    } else if (prevPinState != 0 && currPinState == 0) {
        // Jack plugged in
        Command jackInCommand = {2, COMMAND_TYPE_CONNECTED, pinNumber, 0, false};
        g_currentCommand = jackInCommand;
        sendMessage(jackInCommand);
        blinkLED(GREEN_LED_PIN, -1, fast_blink);
        
//        if (!waiting_for_match) {
////          Serial.print("Jack plugged in on ");
////          Serial.print(portName);
////          Serial.print(" pin ");
////          Serial.println(pinNumber);
//          Command jackInCommand = {2, COMMAND_TYPE_CONNECTED, pinNumber, 0, false};
//          g_currentCommand = jackInCommand;
//          sendMessage(jackInCommand);
//          blinkLED(GREEN_LED_PIN, -1, fast_blink);
//        } else {
//          Command jackInCommand = {2, COMMAND_TYPE_CONNECTED, pinNumber, 0, false};
//          g_currentCommand = jackInCommand;
//          sendMessage(jackInCommand);
//          blinkLED(GREEN_LED_PIN, -1, fast_blink);
//        }
        
    }
}

void checkPinsWithFrequency(unsigned long interval) {
    static unsigned long lastCheckTime = 0;

    if (millis() - lastCheckTime >= interval) {
        lastCheckTime = millis();
        
        PinStates currentStates = readAllPins();

        // Process each jack-connected pin only
        for (int j = 0; j < numJackPins; j++) {
            int pinNumber = jackPinsConnected[j];
            uint8_t port = pgm_read_byte(&digital_pin_to_port_PGM[pinNumber]);
            uint8_t bit = pgm_read_byte(&digital_pin_to_bit_mask_PGM[pinNumber]);

            uint8_t previousState, currentState;
            const char* portName;

            switch (port) {
                case 2: previousState = previousStates.portB; currentState = currentStates.portB; portName = "PORTB"; break;
                case 3: previousState = previousStates.portC; currentState = currentStates.portC; portName = "PORTC"; break;
                case 4: previousState = previousStates.portD; currentState = currentStates.portD; portName = "PORTD"; break;
                case 5: previousState = previousStates.portE; currentState = currentStates.portE; portName = "PORTE"; break;
                case 6: previousState = previousStates.portF; currentState = currentStates.portF; portName = "PORTF"; break;
                case 7: previousState = previousStates.portG; currentState = currentStates.portG; portName = "PORTG"; break;
                case 8: previousState = previousStates.portH; currentState = currentStates.portH; portName = "PORTH"; break;
                case 10: previousState = previousStates.portJ; currentState = currentStates.portJ; portName = "PORTJ"; break;
                case 11: previousState = previousStates.portK; currentState = currentStates.portK; portName = "PORTK"; break;
                case 12: previousState = previousStates.portL; currentState = currentStates.portL; portName = "PORTL"; break;
                default: continue; // Skip if port is not recognized
            }

            // Call the function to process the pin state change
            processPinStateChange(previousState, currentState, bit, portName, pinNumber);
        }

        // Update previous state for the next check
        previousStates = currentStates;
    }
}

void blinkLED(int pin, int count, int pause) {
    BlinkInfo* info = (pin == RED_LED_PIN) ? &blinkInfos[0] : &blinkInfos[1];
    info->pin = pin;
    info->count = count;
    info->pause = pause;
    info->lastToggle = millis();
//    info->initialState = digitalRead(pin);  // Store the initial state
    info->ledState = !info->initialState;  // Start with the opposite state
    info->toggleCount = 0;
    digitalWrite(pin, info->ledState);
}

void updateLEDBlink() {
    unsigned long currentMillis = millis();
    
    for (int i = 0; i < 2; i++) {
        BlinkInfo* info = &blinkInfos[i];
        
        if (info->count != 0 && (currentMillis - info->lastToggle >= info->pause)) {
            info->ledState = !info->ledState;
            digitalWrite(info->pin, info->ledState);
            info->lastToggle = currentMillis;
            
            if (info->ledState == info->initialState) {
                info->toggleCount++;
                if (info->count > 0 && info->toggleCount >= info->count) {
                    stopBlinkLED(info->pin);
                }
            }
        }
    }
}

void stopBlinkLED(int pin) {
    BlinkInfo* info = (pin == RED_LED_PIN) ? &blinkInfos[0] : &blinkInfos[1];
    info->count = 0;
    digitalWrite(pin, info->initialState);  // Return to the initial state
}
