#include <Arduino.h>

#define MAX_BUFFER_SIZE 256
#define END_MARKER 0xFF
#define START_MARKER 0xFE

const char* LIBRARY_NAME = "Fundamental";
const char* MODULE_SLUG = "VCO";
const uint8_t COMMAND_TYPE_INITIALIZE = 0x01;
const uint8_t COMMAND_TYPE_PARAMETER = 0x02;

const int POT_PIN = A0;
const int RED_LED_PIN = 8;
const int GREEN_LED_PIN = 9;

struct Command {
    uint8_t commandId;
    uint8_t commandType;
    uint16_t payload1;
    uint16_t payload2;
    bool acknowledged;
};

volatile bool g_initialized = false;
volatile int g_lastPotValue = -1;
Command g_currentCommand = {1, COMMAND_TYPE_INITIALIZE, 0, 0, false};

void setupPins();
void setupModule();
void handlePotentiometer();
void sendMessage(const Command& cmd);
void processIncomingMessage();
void processResponse(const uint8_t* buffer, size_t size);
void moduleInitialized();

void setup() {
    setupPins();
    Serial.begin(230400);
    setupModule();
}

void loop() {
    if (g_initialized) {
        handlePotentiometer();
        if (!g_currentCommand.acknowledged) {
            processIncomingMessage();
        }
    }
}

void setupPins() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);
}

void setupModule() {
    while (!g_initialized) {
        sendMessage(g_currentCommand);
        delay(100);
        processIncomingMessage();
    }
}

void handlePotentiometer() {
    int potValue = analogRead(POT_PIN);
    if (abs(potValue - g_lastPotValue) > 2) {
        g_lastPotValue = potValue;
        Command potCommand = {2, COMMAND_TYPE_PARAMETER, 0, static_cast<uint16_t>(potValue), false};
        g_currentCommand = potCommand;
        sendMessage(potCommand);
    }
}

void sendMessage(const Command& cmd) {
    uint8_t buffer[12];
    buffer[0] = START_MARKER;
    buffer[1] = 8; // length
    buffer[2] = cmd.commandId;
    buffer[3] = cmd.commandType;
    buffer[4] = (cmd.payload1 >> 8) & 0xFF;
    buffer[5] = cmd.payload1 & 0xFF;
    buffer[6] = (cmd.payload2 >> 8) & 0xFF;
    buffer[7] = cmd.payload2 & 0xFF;
    buffer[8] = END_MARKER;
    Serial.write(buffer, 9);
}

void processIncomingMessage() {
    static uint8_t buffer[12];
    static size_t bufferIndex = 0;
    static bool messageStarted = false;

    while (Serial.available()) {
        uint8_t byte = Serial.read();

        if (!messageStarted && byte == START_MARKER) {
            messageStarted = true;
            bufferIndex = 0;
            continue;
        }

        if (messageStarted) {
            buffer[bufferIndex++] = byte;

            if (bufferIndex == 8) {
                if (byte == END_MARKER) {
                    processResponse(buffer, bufferIndex);
                }
                messageStarted = false;
                bufferIndex = 0;
            }

            if (bufferIndex >= MAX_BUFFER_SIZE) {
                messageStarted = false;
                bufferIndex = 0;
            }
        }
    }
}

void processResponse(const uint8_t* buffer, size_t size) {
    uint8_t commandId = buffer[0];
    uint8_t commandType = buffer[1];
    uint16_t payload1 = (buffer[2] << 8) | buffer[3];
    uint16_t payload2 = (buffer[4] << 8) | buffer[5];

    if (g_currentCommand.commandId == commandId &&
        g_currentCommand.commandType == commandType &&
        g_currentCommand.payload1 == payload1 &&
        g_currentCommand.payload2 == payload2) {
        g_currentCommand.acknowledged = true;
        if (commandType == COMMAND_TYPE_INITIALIZE) {
            moduleInitialized();
        }
    }
}

void moduleInitialized() {
    g_initialized = true;
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
}
