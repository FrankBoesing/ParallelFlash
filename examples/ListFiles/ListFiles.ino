#include <ParallelFlash.h>

void setup() {

  // wait for Arduino Serial Monitor
  while (!Serial) ;
  delay(100);
  Serial.println("All Files on Flash chip:");

  if (!ParallelFlash.begin()) {
    error("Unable to access Flash chip");
  }

  ParallelFlash.opendir();
  unsigned int count = 0;
  while (1) {
    char filename[64];
    unsigned long filesize;

    if (ParallelFlash.readdir(filename, sizeof(filename), filesize)) {
      Serial.print("  ");
      Serial.print(filename);
      spaces(20 - strlen(filename));
      Serial.print("  ");
      Serial.print(filesize);
      Serial.print(" bytes");
      Serial.println();
    } else {
      break; // no more files
    }
  }
}

void spaces(int num) {
  for (int i=0; i < num; i++) {
    Serial.print(" ");
  }
}

void loop() {
}

void error(const char *message) {
  while (1) {
    Serial.println(message);
    delay(2500);
  }
}
