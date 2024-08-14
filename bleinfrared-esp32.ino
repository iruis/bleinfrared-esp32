// reference
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/rmt.html#transmit-data
// https://github.com/espressif/esp-idf/blob/v4.3/examples/peripherals/rmt/morse_code/main/morse_code_main.c
#include <driver/rmt.h>

String serialBuffer;
unsigned long serialTime;

// 적외선 신호 정의
// 1. 매직넘버
// 2. LSB first 유무
// 3. nibble 사용 유무 (nibble일 경우 4비트, 아닐경우 8비트)
// 4. 신호 길이
//  4.1. 리드 길이
//  4.2. 신호 1 길이
//  4.3. 신호 0 길이
//  4.4. 종료 길이
// 5. 적외선 데이터
typedef struct __attribute__ ((packed))
{
  int32_t mark;
  int32_t space;
} bit_us_t;

typedef struct __attribute__ ((packed))
{
  bit_us_t start;
  bit_us_t one;
  bit_us_t zero;
  bit_us_t end;
} duration_t;

typedef struct __attribute__ ((packed))
{
  int16_t magic;
  int8_t lsb;
  int8_t nibble;
  duration_t duration;
} header_t;

static void dumpData(const header_t *header, const uint8_t *data, size_t length)
{
  Serial.println("common");
  Serial.print("  magic: ");
  Serial.println(ntohs(header->magic), HEX);
  Serial.print("  lsb: ");
  Serial.println(header->lsb);
  Serial.print("  nibble: ");
  Serial.println(header->nibble);
  Serial.println("duration");
  Serial.print("  start: +");
  Serial.print(ntohl(header->duration.start.mark));
  Serial.print(",-");
  Serial.println(ntohl(header->duration.start.space));
  Serial.print("  one: +");
  Serial.print(ntohl(header->duration.one.mark));
  Serial.print(",-");
  Serial.println(ntohl(header->duration.one.space));
  Serial.print("  zero: +");
  Serial.print(ntohl(header->duration.zero.mark));
  Serial.print(",-");
  Serial.println(ntohl(header->duration.zero.space));
  Serial.print("  end: +");
  Serial.print(ntohl(header->duration.end.mark));
  Serial.print(",-");
  Serial.println(ntohl(header->duration.end.space));
  Serial.print("data: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print(data[i], HEX);

    if (i < length - 1)
    {
      Serial.print(",");
    }
  }
  Serial.println();
}

static inline void rmt_item32_fill(rmt_item32_t *item, int high, int low)
{
  item->level0 = 1;
  item->duration0 = high;
  item->level1 = 0;
  item->duration1 = low;
}

// reference https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/rmt.html
static void writeIR(const header_t *header, const uint8_t *data, size_t length)
{
  size_t index = 0;
  size_t bitCount = length * (header->nibble ? 4 : 8);
  const duration_t *duration = &header->duration;

  rmt_item32_t *items = (rmt_item32_t*)malloc(sizeof(rmt_item32_t) * (bitCount + 2));
  if (!items)
  {
    return;
  }
  dumpData(header, data, length);

  rmt_item32_fill(items + index++, ntohl(duration->start.mark), ntohl(duration->start.space));

  for (int i = 0; i < length; i++)
  {
    uint8_t byte = data[i];
    const uint8_t mask = header->lsb ? 0x01 : 0x08;
    const uint8_t bits = header->nibble ? 4 : 8;

    for (int bit = 0; bit < bits; bit++)
    {
      const bit_us_t bitUs = (byte & mask) ? duration->one : duration->zero;

      rmt_item32_fill(items + index++, ntohl(bitUs.mark), ntohl(bitUs.space));

      if (header->lsb)
      {
        byte >>= 1;
      }
      else
      {
        byte <<= 1;
      }
    }
  }

  rmt_item32_fill(items + index++, ntohl(duration->end.mark), ntohl(duration->end.space));

  digitalWrite(LED_BUILTIN, HIGH);
  ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL_1, items, bitCount + 2, true));
  digitalWrite(LED_BUILTIN, LOW);

  free(items);
}

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define CONTROLLER_MAGIC int16_t(0xCAFE)
#define DEVICE_NAME "Geekble Mini"
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer *pServer = nullptr;
static BLECharacteristic *pTxCharacteristic = nullptr;
static BLECharacteristic *pRxCharacteristic = nullptr;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
public:
  void onConnect(BLEServer *pServer) override
  {
    deviceConnected = true;

    Serial.println("onConnect");
  }

  void onDisconnect(BLEServer *pServer) override
  {
    deviceConnected = false;

    Serial.println("onDisconnect");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    rxValue = pCharacteristic->getValue();
  }

  String value()
  {
    return std::move(rxValue);
  }

private:
  String rxValue;
};

MyCallbacks *pCallbacks = nullptr;

void setup()
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  pCallbacks = new MyCallbacks();

  BLEDevice::init(DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(pCallbacks);

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("Waiting a client connection to notify...");

  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_6, RMT_CHANNEL_1);
  config.tx_config.carrier_en = true;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

static void processRx()
{
  String rxValue = pCallbacks->value();
  if (!rxValue.length())
  {
    return;
  }

  const uint8_t *data = (const uint8_t *)rxValue.c_str();
  unsigned int length = rxValue.length();

  Serial.print("rx length: ");
  Serial.print(length);
  Serial.print(", minimum: ");
  Serial.println(sizeof(header_t));

  header_t *header = (header_t*)data;
  if (sizeof(header_t) >= length)
  {
    return;
  }

  Serial.print("magic: ");
  Serial.println(CONTROLLER_MAGIC == int16_t(ntohs(header->magic)) ? "ok" : "nok");

  Serial.print("net: ");
  Serial.print(data[0], HEX);
  Serial.println(data[1], HEX);

  Serial.print("ntohs: ");
  Serial.println(ntohs(header->magic), HEX);

  if (int16_t(ntohs(header->magic)) != CONTROLLER_MAGIC)
  {
    return;
  }
  writeIR(header, data + sizeof(header_t), length - sizeof(header_t));
}

static void processSerial()
{
  if (serialBuffer.length() % 2)
  {
    return;
  }
  size_t length = serialBuffer.length() / 2;
  uint8_t *data = (uint8_t*)malloc(length);
  if (!data)
  {
    return;
  }
  for (int i = 0; i < length; i++)
  {
    char hi = serialBuffer[i * 2 + 0];
    char lo = serialBuffer[i * 2 + 1];
  }

  free(data);
}

void loop()
{
  // 시리얼로 수신되는 데이터 확인 및 처리
  if (Serial.available())
  {
    // 현재 기준 100 밀리초 이전에 수신 된 데이터는 버림
    unsigned long uptime = millis();
    if (serialTime > uptime + 100)
    {
      serialBuffer.clear();
    }
    serialTime = uptime;

    char ch = Serial.read();
    if (ch == '\n')
    {
      processSerial();
      serialBuffer.clear();
    }
    else
    {
      serialBuffer += ch;
    }
  }

  if (deviceConnected)
  {
    processRx();
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);

    pServer->startAdvertising();
    Serial.println("start advertising");

    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }
}
