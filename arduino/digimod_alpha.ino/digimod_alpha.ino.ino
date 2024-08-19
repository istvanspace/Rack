#define MAX_BUFFER_SIZE 256  // Adjust size based on your needs
#define MAX_PAYLOAD_SIZE 20  // Maximum size for each payload
#define END_MARKER 0xFF  // Define the end-of-message marker
#define START_MARKER 0xFE // Define the start-of-message marker

const int buttonPin = 2;  // the number of the pushbutton pin
const int redLedPin = 8;
const int greenLedPin = 9;

bool pressed = false;  // To track button state
bool initialized = false;  // To track if initialization has occurred

struct Command {
    uint8_t commandId;
    uint8_t commandType;
    const char* payload1;  
    const char* payload2;
    bool acknowledged;
};

const uint8_t CommandType_Initialize = 0x01;  // Added missing semicolon

Command initCommand = {1, 0x01, "Fundamental", "VCO",  false};  // Initialize the command structure
Command current_Command = initCommand;


void blinkRedPin(int blink_count, int pause) {
  int initialState = digitalRead(redLedPin);
  
  for (int i = 0; i < blink_count; i++) {
      digitalWrite(redLedPin, HIGH);
      delay(pause);  // Longer delay for clearer debugging
      digitalWrite(redLedPin, LOW);
      delay(pause);
  }

  digitalWrite(redLedPin, initialState);
}

void blinkGreenPin(int blink_count, int pause) {
  int initialState = digitalRead(greenLedPin);
  
  for (int i = 0; i < blink_count; i++) {
      digitalWrite(greenLedPin, HIGH);
      delay(pause);  // Longer delay for clearer debugging
      digitalWrite(greenLedPin, LOW);
      delay(pause);
  }
  
  digitalWrite(greenLedPin, initialState);
}

void sendMessage(Command new_Command) {
    uint8_t commandId = new_Command.commandId;
    uint8_t commandType = new_Command.commandType;
    const char* payload1 = new_Command.payload1;
    const char* payload2 = new_Command.payload2;

    uint8_t payload1Length = strlen(payload1);
    uint8_t payload2Length = strlen(payload2);

    if (payload1Length >= MAX_PAYLOAD_SIZE || payload2Length >= MAX_PAYLOAD_SIZE) {
        return;
    }

    uint8_t buffer[MAX_BUFFER_SIZE];
    uint8_t* ptr = buffer;

    *ptr++ = START_MARKER;

    uint16_t messageLength = 4 + payload1Length + payload2Length;  // 4 for commandId, commandType, and payload lengths
    *ptr++ = (messageLength >> 8) & 0xFF;  // High byte of length
    *ptr++ = messageLength & 0xFF;  // Low byte of length

    *ptr++ = commandId;
    *ptr++ = commandType;
    *ptr++ = payload1Length;  
    *ptr++ = payload2Length;  

    memcpy(ptr, payload1, payload1Length);
    ptr += payload1Length;

    memcpy(ptr, payload2, payload2Length);
    ptr += payload2Length;

    *ptr++ = END_MARKER;

    uint16_t totalLength = ptr - buffer;
    Serial.write(buffer, totalLength);
    Serial.flush();
}

void processResponse(const uint8_t* buffer, size_t size, Command last_Command) {
    if (size < 4) {
        blinkRedPin(4, 100);
        return;
    }

    uint8_t commandId = buffer[0];
    uint8_t commandType = buffer[1];
    uint8_t payload1Length = buffer[2];
    uint8_t payload2Length = buffer[3];

    if (size < 4 + payload1Length + payload2Length) {
        blinkRedPin(3, 500);
        return;
    }

    char payload1[MAX_PAYLOAD_SIZE] = {0};
    char payload2[MAX_PAYLOAD_SIZE] = {0};

    memcpy(payload1, buffer + 4, payload1Length);
    memcpy(payload2, buffer + 4 + payload1Length, payload2Length);

    // Check payloads for correctness
    if (last_Command.commandId == commandId && 
        strcmp(last_Command.payload1, payload1) == 0 && 
        strcmp(last_Command.payload2, payload2) == 0) {
        current_Command.acknowledged = true;
        initialized = true;  // Mark as initialized if acknowledgment is successful
        digitalWrite(greenLedPin, HIGH);  // Turn on green LED
        digitalWrite(redLedPin, LOW);  // Turn off red LED
    } else {
        blinkGreenPin(3,100);
        sendMessage(last_Command);
    }
}


void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(redLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);
    Serial.begin(115200);
}

void loop() {
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
                // First two bytes after START_MARKER represent message length
                expectedLength = (buffer[0] << 8) | buffer[1];
                continue;
            }

            if (bufferIndex == expectedLength + 3) {  // +3 for length bytes and END_MARKER
                if (byte == END_MARKER) {
                    // Process the message
                    blinkGreenPin(2, 100);
                    processResponse(buffer + 2, bufferIndex - 3, current_Command);  // Skip length bytes and END_MARKER
                } else {
                    // Invalid END_MARKER
                    blinkRedPin(3, 100);
                }

                messageStarted = false;
                bufferIndex = 0;
                break;
            }
        }

        // Prevent buffer overflow
        if (bufferIndex >= MAX_BUFFER_SIZE) {
            blinkRedPin(5, 100);  // Indicate buffer overflow
            messageStarted = false;
            bufferIndex = 0;
        }
    }

    int buttonState = digitalRead(buttonPin);
    if (buttonState == LOW && !pressed && !initialized) {
        sendMessage(initCommand);
        pressed = true;
        delay(100);  // debounce delay
    } else if (buttonState == HIGH && pressed) {
        pressed = false;
        delay(100);  // debounce delay
    }
    
    if (!initialized) {
        digitalWrite(redLedPin, HIGH);  // Turn on red LED
        digitalWrite(greenLedPin, LOW);  // Turn off green LED
    } else {
        digitalWrite(redLedPin, LOW);  // Turn off red LED
        digitalWrite(greenLedPin, HIGH);  // Turn on green LED
    }
}
