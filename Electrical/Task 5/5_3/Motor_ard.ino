// SECOND ARDUINO: MOTOR CONTROLLER WITH COMMAND ISOLATION
//For further improvment we can compare the expected encoder behaviour with the actual one from motors

String inputString = "";     // Buffer for incoming command
bool receiving = false;      // Are we inside a `<...>` block?

// Motor pins
const int M1_IN1 = 13;
const int M1_IN2 = 12;
const int M1_EN  = 11;
const int M2_IN1 = 9;
const int M2_IN2 = 8;
const int M2_EN  = 10;

void setup() {
  Serial.begin(9600);        // Start serial communication
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  stopMotors();              // Ensure motors start off
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();   // Read incoming char

    if (c == '<') {
      // start of command
      inputString = "";       // Reset command buffer
      receiving = true;       // Start receiving command
    }

    else if (c == '>' && receiving) {
      // end of command
      receiving = false;              // Stop receiving
      inputString.trim();             // Clean up any whitespace
      Serial.print("Received: ");
      Serial.println(inputString);    // Print full command for debugging
      parseAndRun(inputString);       // Parse and execute
    }

    else if (receiving) {
      // buffer only while inside <…>
      inputString += c;       // Append character to buffer
    }

    // any data outside <…> is ignored
  }
}

void parseAndRun(String cmd) {
  // find the last "SPD:" in the string
  int spdIndex   = cmd.lastIndexOf("SPD:");
  int commaIndex = cmd.indexOf(',', spdIndex);

  if (spdIndex == -1 || commaIndex == -1) {
    Serial.println("Invalid CMD");  // Command doesn't match expected format
    return;
  }

  int speedVal = cmd.substring(spdIndex + 4, commaIndex).toInt(); // Extract speed
  String dir   = cmd.substring(commaIndex + 1);                   // Extract direction
  dir.trim();                                                     // Remove any extra spaces

  moveMotors(speedVal, dir);     // Call motor function
}

void moveMotors(int speed, String dir) {
  Serial.print("Running ");
  Serial.print(dir);
  Serial.print(" at ");
  Serial.println(speed);

  speed = constrain(speed, 0, 255);    // Ensure speed is within PWM range

  if (dir == "STOP") {
    stopMotors();                      // Stop both motors
  }

  if (dir == "FWD") {
    // Move both motors forward
    digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);  analogWrite(M1_EN, speed);
    digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);  analogWrite(M2_EN, speed);
  }

  else if (dir == "BACK") {
    // Move both motors backward
    digitalWrite(M1_IN1, LOW);  digitalWrite(M1_IN2, HIGH); analogWrite(M1_EN, speed);
    digitalWrite(M2_IN1, LOW);  digitalWrite(M2_IN2, HIGH); analogWrite(M2_EN, speed);
  }

  else if (dir == "LEFT") {
    // Left turn (right motor forward, left motor backward)
    digitalWrite(M1_IN1, LOW);  digitalWrite(M1_IN2, HIGH); analogWrite(M1_EN, speed);
    digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);  analogWrite(M2_EN, speed);
  }

  else if (dir == "RIGHT") {
    // Right turn (left motor forward, right motor backward)
    digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);  analogWrite(M1_EN, speed);
    digitalWrite(M2_IN1, LOW);  digitalWrite(M2_IN2, HIGH); analogWrite(M2_EN, speed);
  }
}

void stopMotors() {
  // Stop motor 1
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, 0);

  // Stop motor 2
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  analogWrite(M2_EN, 0);
}
