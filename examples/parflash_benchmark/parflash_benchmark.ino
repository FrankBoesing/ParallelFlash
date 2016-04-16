#include <ParallelFlash.h>
#include <FastCRC.h>

FastCRC32 CRC32;
char buf[1024]  __attribute__ ((aligned (4)));

const char* filename = "rain.aac";

void setup() {
  // put your setup code here, to run once:
  delay(1000);

  // Start ParallelFlash
  if (!ParallelFlash.begin()) {
    while (1) {
      Serial.println ("Cannot access Flash chip");
      delay (1000);
    }
  }

  Serial.printf ("Flash initialzed. F_CPU = %d\n", F_CPU);
  ParallelFlashFile ff = ParallelFlash.open(filename);

  uint32_t sz = ff.size();
  uint32_t pos = ff.getFlashAddress();

  if (!ff) {
    while (1) {
      Serial.printf("%s not found.\n", filename);
      delay (1000);
    }
  }

  Serial.printf("%s: Size:%d Position:%d\n", filename, sz, pos);
  delay(50);

  unsigned int m, count, r, crc;
  count = 0;
  m = millis();
  while (count < sz) {
    count += ff.read(buf, sizeof(buf));
  }
  m = millis() - m;
  Serial.printf("%d Bytes in %d Milliseconds = %f Megabyte / Sec \n", count, m, (sz / 1024.0 / 1024.0) / (m / 1000.0));
  ff.seek(0);
  count = 0;

  count += ff.read(buf, sizeof(buf));
  CRC32.crc32((const uint8_t *) buf, sizeof(buf));

  while (count < sz) {
    r = ff.read(buf, sizeof(buf));
    crc = CRC32.crc32_upd((const uint8_t *) buf, r);
    count += r;
  }
  Serial.printf("CRC:0x%X\n", crc);
}

void loop() {

}


