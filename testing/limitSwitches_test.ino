void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);

  // Set up digital pins as inputs
  pinMode(35, INPUT);
  pinMode(37, INPUT);
  pinMode(39, INPUT);
  pinMode(41, INPUT);
}

void loop() {
  // Print all digital values on the same line
  Serial.print("D35: ");
  Serial.print(digitalRead(35));
  Serial.print("  ");

  Serial.print("D37: ");
  Serial.print(digitalRead(37));
  Serial.print("  ");

  Serial.print("D39: ");
  Serial.print(digitalRead(39));
  Serial.print("  ");

  Serial.print("D41: ");
  Serial.print(digitalRead(41));
  Serial.println();  // Move to the next line after all values are printed

  delay(50);  // Short delay to allow constant updates, adjust if needed
}
