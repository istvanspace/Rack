#include <Arduino.h>

// Constants
#define MAX_BUFFER_SIZE 256
#define MAX_PAYLOAD_SIZE 20
#define END_MARKER 0xFF
#define START_MARKER 0xFE

// Module Settings
const char* LIBRARY_NAME = "Fundamental";
const char* MODULE_SLUG = "VCO";

// Arduino Settings
const int BUTTON_PIN = 2;
const int RED_LED_PIN = 8;
const int GREEN_LED_PIN = 9;
const int POT_PIN = A0;
const int slow_blink = 250;
const int fast_blink = 10;

const uint8_t COMMAND_TYPE_INITIALIZE = 0x01;
const uint8_t COMMAND_TYPE_PARAMETER = 0x02;

// Structs
struct Command {
    uint8_t commandId;
    uint8_t commandType;
    const char* payload1;
    const char* payload2;
    bool acknowledged;
};

// Global variables
bool g_initialized = false;
int g_lastPotValue = -1;
Command g_currentCommand = {1, COMMAND_TYPE_INITIALIZE, LIBRARY_NAME, MODULE_SLUG, false};

// Function prototypes
void setupPins();
void setupModule();
void handlePotentiometer();
void blinkLED(int pin, int count, int pause);
void sendMessage(const Command& cmd);
void processIncomingMessage();
void processResponse(const uint8_t* buffer, size_t size, const Command& lastCommand);
void moduleInitialized();

void setup() {
    setupPins();
    updateLEDStatus();
    Serial.begin(230400);
    setupModule();
}

void loop() {
    if (g_initialized) {
      
        handlePotentiometer();
        if (!g_currentCommand.acknowledged){
          processIncomingMessage();
        }
        
    }
}

void setupPins() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);
}

void setupModule() {
   while (!g_initialized) {
      sendMessage(g_currentCommand);
      delay(500);
      processIncomingMessage();
    }
}

void handlePotentiometer() {
    int potValue = analogRead(POT_PIN);
    if (abs(potValue - g_lastPotValue) > 2) {
        g_lastPotValue = potValue;
        char potValueStr[6];
        snprintf(potValueStr, sizeof(potValueStr), "%d", potValue);
        
        Command potCommand = {2, COMMAND_TYPE_PARAMETER, "0", potValueStr, false};
        g_currentCommand = potCommand;
        sendMessage(potCommand);
        blinkLED(RED_LED_PIN, 1, fast_blink);
    }
}

void blinkLED(int pin, int count, int pause) {
    int initialState = digitalRead(pin);
    for (int i = 0; i < count; i++) {
        digitalWrite(pin, HIGH);
        delay(pause);
        digitalWrite(pin, LOW);
        delay(pause);
    }
    digitalWrite(pin, initialState);
}

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
                    blinkLED(GREEN_LED_PIN, 2, 100);
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

void processResponse(const uint8_t* buffer, size_t size, const Command& lastCommand) {
    if (size < 4) {
        blinkLED(RED_LED_PIN, 4, slow_blink);
        return;
    }
    
    uint8_t commandId = buffer[0];
    uint8_t commandType = buffer[1];
    uint8_t payload1Length = buffer[2];
    uint8_t payload2Length = buffer[3];
    
    if (size < 4 + payload1Length + payload2Length) {
        blinkLED(RED_LED_PIN, 4, slow_blink);
        return;
    }
    
    char payload1[MAX_PAYLOAD_SIZE] = {0};
    char payload2[MAX_PAYLOAD_SIZE] = {0};
    
    memcpy(payload1, buffer + 4, payload1Length);
    memcpy(payload2, buffer + 4 + payload1Length, payload2Length);
    
    if (lastCommand.commandId == commandId && 
        strcmp(lastCommand.payload1, payload1) == 0 && 
        strcmp(lastCommand.payload2, payload2) == 0) {
        g_currentCommand.acknowledged = true;
        if (lastCommand.commandType == COMMAND_TYPE_INITIALIZE) {
            moduleInitialized();
        }
    } else {
        blinkLED(GREEN_LED_PIN, 3, fast_blink);
        sendMessage(lastCommand);
    }
}

void moduleInitialized() {
  g_initialized = true;
  updateLEDStatus();
}

void updateLEDStatus() {
    digitalWrite(RED_LED_PIN, !g_initialized);
    digitalWrite(GREEN_LED_PIN, g_initialized);
}
