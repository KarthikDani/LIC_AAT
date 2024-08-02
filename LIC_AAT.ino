#include <math.h>
#define potPin A0

uint16_t icr = 0xffff;
String line;
int resolution = 16;
float dutyCycle = 0;
uint16_t value = 0;

// Mode variables
bool potentiometerMode = true; // true for potentiometer mode, false for serial monitor mode
const float potThreshold = 0; // Threshold to consider the potentiometer is grounded

void setup() {
  Serial.begin(9600);
  printIntroduction();
  initializePWM();
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  float potVoltage = (potValue / 1023.0) * 5.0;

  // Determine mode based on potentiometer voltage
  if (potVoltage <= potThreshold) {
    // Switch to Serial Monitor Mode if the potentiometer reads 5V or close to it
    if (potentiometerMode) {
      potentiometerMode = false;
      Serial.println("Switched to Serial Monitor Mode.");
      // Clear serial buffer
      while (Serial.available() > 0) {
        Serial.read();
      }
    }
  } else {
    // Switch to Potentiometer Mode if the potentiometer is not at 5V
    if (!potentiometerMode) {
      potentiometerMode = true;
      Serial.println("Switched to Potentiometer Mode.");
    }
  }

  if (potentiometerMode) {
    // In potentiometer mode, read the potentiometer and update duty cycle
    readPotentiometer();
  } else {
    // In serial monitor mode, process serial input
    if (Serial.available() > 0) {
      processSerialInput();
    }
  }
}

void printIntroduction() {
  // Print introduction and instructions to the Serial Monitor.
  Serial.println("Interactive 16-bit PWM");

  Serial.println("1. Change Resolution: Enter a to i for 8 to 16 bit resolution | Default: 16 bit");
  Serial.println("2. Change Duty Cycle: Enter 0 to 100 %");
  Serial.println("3. Switch to Serial Monitor Mode automatically when Potentiometer Pin is GROUNDED!");
}

void initializePWM() {
  // Initialize PWM settings and configure the PWM based on the default resolution.
  OCR1A = 0;
  OCR1B = 0;
  setupPWM();
}

void processSerialInput() {
  // Process commands received from the Serial Monitor.
  line = Serial.readString();
  line.trim();

  if (line.length() > 0) {
    Serial.println();
    Serial.print("Command: ");
    Serial.println(line);

    if (isDutyCycleCommand(line) && 
        !isResolutionCommand(line)) {
      handleDutyCycleCommand(line);
    } 
    else if (isResolutionCommand(line)) {
      handleResolutionCommand(line[0]);
    } 
    else {
      Serial.println("Enter a value between 0 and 100 for duty cycle or 'a' to 'i' for resolution.");
    } 
  } 
  else {
    Serial.println("Received empty command.");
  }
}

bool isResolutionCommand(String line) {
  // Check if the command is a single character for changing resolution.
  if (line.length() == 1) {
    char command = line[0];
    return command >= 'a' && command <= 'i';
  }
  return false;
}

bool isDecimalOrInteger(String input) {
  // Trim any leading or trailing whitespace
  input.trim();

  // Check if the input is empty
  if (input.length() == 0) {
    return false;
  }

  // Check if the input contains a decimal point
  bool isDecimal = false;
  bool hasDigit = false;

  for (int i = 0; i < input.length(); i++) {
    char c = input[i];
    
    // Check if the character is a digit
    if (isdigit(c)) {
      hasDigit = true;
    }
    // Check if the character is a decimal point
    else if (c == '.') {
      // If there's already a decimal point, return false (invalid number)
      if (isDecimal) {
        return false;
      }
      isDecimal = true;
    }
    // If the character is neither a digit nor a decimal point, return false
    else {
      return false;
    }
  }

  // Ensure there's at least one digit in the input
  if (!hasDigit) {
    return false;
  }

  // If the function hasn't returned false yet, the input is a valid number
  return true;
}

bool isDutyCycleCommand(String line) {
  // Check if the command is a valid duty cycle value between 0 and 100.
  if (line.length() > 0 && isDecimalOrInteger(line)) {
    float value = line.toFloat();
    return value >= 0.0 && value <= 100.0;
  }
  return false;
}

void handleResolutionCommand(char command) {
  // Handle resolution change commands for character indicating the desired resolution.
  if ('a' <= command && command <= 'i') {
    resolution = 8 + command - 'a';
    Serial.print("Resolution changed to: ");
    Serial.print(resolution);
    Serial.println(" bit");

    setupPWM();
    updatePWMValue();
  }
}

void handleDutyCycleCommand(String line) {
  dutyCycle = line.toFloat();
  if (dutyCycle >= 0.0 && dutyCycle <= 100.0) {
    //dutyCycle = 1 - dutyCycle;
    updatePWMValue();
  } else {
    Serial.println("Invalid Duty Cycle. Enter a value between 0.0 and 100.0.");
  }
}

void updatePWMValue() {
  // Update the PWM value based on the current duty cycle and resolution.

  // Calculate the maximum value for the current resolution
  uint16_t maxValue = calculateICR(resolution);

  // Calculate the PWM value and cap it to the maximum value
  value = round(dutyCycle * maxValue / 100.0);
  value = min(value, maxValue); // Ensure value does not exceed the maxValue

  Serial.print("Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.println(" %");

  Serial.print("Voltage: ");
  Serial.print(dutyCycle * 5 / 100);
  Serial.print("V");

  Serial.print(" | Value: ");
  Serial.print(value);

  Serial.print(" @ ");
  Serial.print(resolution);
  Serial.println(" bit resolution");

  analogWritePin9();
}

void setupPWM() {
  // Set up the PWM with the current resolution.

  icr = calculateICR(resolution);
  configurePWM();
  //Serial.print("ICR1: ");
  //Serial.println(icr);
}

uint16_t calculateICR(int resolution) {
  // Calculate the ICR value based on the desired resolution (8 to 16 bits)
  if (resolution < 8 || resolution > 16) {
    Serial.println("Invalid resolution value. Defaulting to 8 bits.");
    resolution = 8;
  }

  switch (resolution) {
    case 16: return 0xffff;
    case 15: return 0x7fff;
    case 14: return 0x3fff;
    case 13: return 0x1fff;
    case 12: return 0x0fff;
    case 11: return 0x07ff;
    case 10: return 0x03ff;
    case  9: return 0x01ff;
    case  8: return 0x00ff;
    default: return 0x00ff; // Default case to handle unexpected values
  }
}

void configurePWM() {
  // Configure the PWM settings based on the current ICR value and resolution.
  DDRB |= _BV(PB1) | _BV(PB2);       // Set pins as outputs
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) // Non-inverting PWM
           | _BV(WGM11);             // Mode 14: Fast PWM, TOP=ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS11);              // Prescaler 1
  ICR1 = icr;                        // TOP counter value
}

void analogWritePin9() {
  // Write the PWM value to pin 9 (OC1A).
  if (value == 0) {
    TCCR1A &= ~_BV(COM1A1);  // Disconnect PWM for pin 9 (OC1A)
    PORTB &= ~_BV(PB1);      // Set pin 9 to LOW using direct port manipulation
  } else {
    TCCR1A |= _BV(COM1A1);   // Reconnect PWM for pin 9 (OC1A)
    OCR1A = value;           // Set PWM duty cycle
  }
}

void readPotentiometer() {
  // Read the value from the potentiometer and update the duty cycle if changed.
  int potValue = analogRead(potPin);
  float potDutyCycle = map(potValue, 0, 1023, 0, 100);

  if (potDutyCycle != dutyCycle) {
    dutyCycle = potDutyCycle;
    updatePWMValue();
  }
}