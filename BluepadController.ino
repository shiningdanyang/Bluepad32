//Board = DOIT ESP32 DEVKIT V1

#include <Bluepad32.h>
#include <uni.h>

#define DEBUG false
bool scanMode = false;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
ControllerPtr gamepadLegacy;

static const char* controller_addr_string = "00:1f:e2:7e:cc:6a";
//
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      ControllerProperties properties = ctl->getProperties();
      Serial.print("scanMode = ");
      Serial.print((scanMode) ? "true" : "false");
      Serial.printf(", model: %s, VID=0x%04x, PID=0x%04x, addr=%02x:%02x:%02x:%02x:%02x:%02x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id, properties.btaddr[0], properties.btaddr[1], properties.btaddr[2], properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      Serial.printf("CALLBACK: Connected, index=%d ", i);
      ctl->setColorLED(255, 0, 0);
      ctl->playDualRumble(0 /* delayedStartMs */, 1000 /* durationMs */, 0x80 /* weakMagnitude */,
                          0x40 /* strongMagnitude */);
      BP32.enableNewBluetoothConnections(false);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Connected wrong controller");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void processGamepad(ControllerPtr ctl) {
  if (ctl->miscSystem()) {
    digitalWrite(2, !digitalRead(2));

    static int led = 0;
    led++;
    ctl->setPlayerLEDs(led & 0x0f);

    ctl->playDualRumble(0 /* delayedStartMs */, 1000 /* durationMs */, 0x80 /* weakMagnitude */,
                        0x40 /* strongMagnitude */);

    static int colorIdx = 0;
    switch (colorIdx % 3) {
      case 0:
        ctl->setColorLED(255, 0, 0);
        break;
      case 1:
        ctl->setColorLED(0, 255, 0);
        break;
      case 2:
        ctl->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  encodeGamepad(ctl);
}


void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

void encodeGamepad(ControllerPtr ctl) {
  uint8_t byte[9];

  (ctl->dpad() & 0b1000) ? (byte[0] |= 0x01 << 7) : (byte[0] &= ~(0x01 << 7));
  (ctl->dpad() & 0b0010) ? (byte[0] |= 0x01 << 6) : (byte[0] &= ~(0x01 << 6));
  (ctl->dpad() & 0b0100) ? (byte[0] |= 0x01 << 5) : (byte[0] &= ~(0x01 << 5));
  (ctl->dpad() & 0b0001) ? (byte[0] |= 0x01 << 4) : (byte[0] &= ~(0x01 << 4));
  (ctl->miscStart()) ? (byte[0] |= 0x01 << 3) : (byte[0] &= ~(0x01 << 3));
  (ctl->thumbR()) ? (byte[0] |= 0x01 << 2) : (byte[0] &= ~(0x01 << 2));
  (ctl->thumbL()) ? (byte[0] |= 0x01 << 1) : (byte[0] &= ~(0x01 << 1));
  (ctl->miscSelect()) ? (byte[0] |= 0x01 << 0) : (byte[0] &= ~(0x01 << 0));

  (ctl->x()) ? (byte[1] |= 0x01 << 7) : (byte[1] &= ~(0x01 << 7));
  (ctl->a()) ? (byte[1] |= 0x01 << 6) : (byte[1] &= ~(0x01 << 6));
  (ctl->b()) ? (byte[1] |= 0x01 << 5) : (byte[1] &= ~(0x01 << 5));
  (ctl->y()) ? (byte[1] |= 0x01 << 4) : (byte[1] &= ~(0x01 << 4));
  (ctl->r1()) ? (byte[1] |= 0x01 << 3) : (byte[1] &= ~(0x01 << 3));
  (ctl->l1()) ? (byte[1] |= 0x01 << 2) : (byte[1] &= ~(0x01 << 2));
  (ctl->r2()) ? (byte[1] |= 0x01 << 1) : (byte[1] &= ~(0x01 << 1));
  (ctl->l2()) ? (byte[1] |= 0x01 << 0) : (byte[1] &= ~(0x01 << 0));

  byte[2] = map(ctl->axisY(), -511, 512, 0, 255);
  byte[3] = map(ctl->axisX(), -511, 512, 0, 255);
  byte[4] = map(ctl->axisRY(), -511, 512, 0, 255);
  byte[5] = map(ctl->axisRX(), -511, 512, 0, 255);
  byte[6] = map(ctl->brake(), 0, 1023, 0, 255);
  byte[7] = map(ctl->throttle(), 0, 1023, 0, 255);

  byte[8] = 0;

  for (int i = 0; i < 8; i++) {
    byte[8] += byte[i];
  }

  if (DEBUG || scanMode) {
    ControllerProperties properties = ctl->getProperties();
    Serial.print("scanMode = ");
    Serial.print((scanMode) ? "true" : "false");
    Serial.printf(", model: %s, VID=0x%04x, PID=0x%04x, addr=%02x:%02x:%02x:%02x:%02x:%02x, ", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id, properties.btaddr[0], properties.btaddr[1], properties.btaddr[2], properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);

    Serial.print((int)byte[0]);
    Serial.print(" ");
    Serial.print((int)byte[1]);
    Serial.print(" ");
    Serial.print((int)byte[2]);
    Serial.print(" ");
    Serial.print((int)byte[3]);
    Serial.print(" ");
    Serial.print((int)byte[4]);
    Serial.print(" ");
    Serial.print((int)byte[5]);
    Serial.print(" ");
    Serial.print((int)byte[6]);
    Serial.print(" ");
    Serial.print((int)byte[7]);
    Serial.print(" ");
    Serial.print((int)byte[8]);
    Serial.print(" ");
    Serial.print(digitalRead(0));
    Serial.println();
  }

  // Serial2.write("<");
  // Serial2.write("<");
  // Serial2.write("<");
  // Serial2.write("<");
  // for (int i = 0; i < 9; i++) {
  //   Serial2.write(byte[i]);
  // }
  // Serial2.write(">");
  // Serial2.write(">");
  // Serial2.write(">");
  // Serial2.write(">");
  
  Serial.write("<");
  Serial.write("<");
  Serial.write("<");
  Serial.write("<");
  for (int i = 0; i < 9; i++) {
    Serial.write(byte[i]);
  }
  Serial.write(">");
  Serial.write(">");
  Serial.write(">");
  Serial.write(">");

  Serial2.write("<");
  Serial2.write("<");
  Serial2.write("<");
  Serial2.write("<");
  for (int i = 0; i < 9; i++) {
    Serial2.write(byte[i]);
  }
  Serial2.write(">");
  Serial2.write(">");
  Serial2.write(">");
  Serial2.write(">");
}

void setup() {
  Serial.begin(19200);
  Serial2.begin(19200);
  pinMode(0, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  delay(1000);
  if (!digitalRead(0)) {
    scanMode = true;
    for (int i = 0; i < 5; i++) {
      digitalWrite(2, !digitalRead(2));
      delay(100);
    }
  }
  uni_bt_allowlist_remove_all();
  if (!scanMode) {
    bd_addr_t controller_addr;
    sscanf_bd_addr(controller_addr_string, controller_addr);
    uni_bt_allowlist_add_addr(controller_addr);
  }
  uni_bt_allowlist_set_enabled(!scanMode);
  uni_bt_allowlist_list();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  delay(1000);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
}
