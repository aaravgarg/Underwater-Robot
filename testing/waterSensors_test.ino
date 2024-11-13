void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Print all analog values on the same line
  for (int i = 0; i <= 5; i++) {
    int analogValue = analogRead(i);  // Read analog value from pin A0 to A5
    Serial.print("A");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(analogValue);
    Serial.print("  ");  // Separate each value with spaces for readability
  }

  Serial.println();  // Move to the next line after all values are printed

  delay(100);  // Short delay to allow constant updates, adjust if needed
}
