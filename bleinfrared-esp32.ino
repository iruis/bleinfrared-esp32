// reference
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/rmt.html#transmit-data
// https://github.com/espressif/esp-idf/blob/v4.3/examples/peripherals/rmt/morse_code/main/morse_code_main.c
#include <driver/rmt.h>

#define BUFFER_MAX 512
char serialBuffer[BUFFER_MAX];
size_t serialBufferLength = 0;

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
  bit_us_t lead;
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
  Serial.print(ntohl(header->duration.lead.mark));
  Serial.print(",-");
  Serial.println(ntohl(header->duration.lead.space));
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

  rmt_item32_fill(items + index++, ntohl(duration->lead.mark), ntohl(duration->lead.space));

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

  ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL_1, items, bitCount + 2, true));

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
    digitalWrite(LED_BUILTIN, HIGH);
  }

  void onDisconnect(BLEServer *pServer) override
  {
    deviceConnected = false;

    Serial.println("onDisconnect");
    digitalWrite(LED_BUILTIN, LOW);
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
  digitalWrite(LED_BUILTIN, LOW);

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

  if (int16_t(ntohs(header->magic)) != CONTROLLER_MAGIC)
  {
    return;
  }
  writeIR(header, data + sizeof(header_t), length - sizeof(header_t));
}

static uint8_t decodeHex(char hex)
{
  if (hex >= '0' && hex <= '9')
  {
    return hex - '0';
  }
  if (hex >= 'A' && hex <= 'F')
  {
    return hex - 'A' + 10;
  }
  if (hex >= 'a' && hex <= 'f')
  {
    return hex - 'a' + 10;
  }
  throw "not a hexadecimal character";
}

static void processSerial()
{
  if (serialBufferLength % 2)
  {
    Serial.println("not divide by 2");
    return;
  }
  for (size_t i = 0; i < serialBufferLength; i++)
  {
    const char ch = serialBuffer[i];

    if (!(ch >= '0' && ch <= '9') && !(ch >= 'A' && ch <= 'F') && !(ch >= 'a' && ch <= 'f'))
    {
      Serial.println("not hex string");
      Serial.println(serialBuffer);
      
      for (int rep = 0; rep < i; rep++)
      {
        Serial.print(" ");
      }
      Serial.print("^");

      return;
    }
  }

  size_t length = serialBufferLength / 2;
  if (length <= sizeof(header_t))
  {
    Serial.println("incorrect data");
    return;
  }
  uint8_t *data = (uint8_t*)malloc(length);
  if (!data)
  {
    return;
  }
  // HEX -> BINARY는 아직 미구현
  for (int i = 0; i < length; i++)
  {
    char hi = serialBuffer[i * 2 + 0];
    char lo = serialBuffer[i * 2 + 1];

    data[i] = decodeHex(hi) << 4 | decodeHex(lo);
  }
  writeIR(reinterpret_cast<header_t*>(data), data + sizeof(header_t), length - sizeof(header_t));

  free(data);
}

void loop()
{
  // 시리얼로 수신되는 데이터 확인 및 처리
  if (Serial.available())
  {
    if (serialBufferLength >= BUFFER_MAX)
    {
      serialBufferLength = 0;
    }

    char ch = Serial.read();
    if (ch == '\r' || ch == '\n')
    {
      if (serialBufferLength == 0)
      {
        return;
      }

      processSerial();
      serialBufferLength = 0;
    }
    else
    {
      serialBuffer[serialBufferLength++] = ch;
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
