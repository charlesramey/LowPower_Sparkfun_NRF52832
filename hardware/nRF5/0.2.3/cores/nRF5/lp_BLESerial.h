#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

#include <Arduino.h>
#include <BLEPeripheral.h>

class lp_BLESerial : public BLEPeripheral, public Stream {
  public:
    lp_BLESerial(size_t txBufferSize = DEFAULT_SEND_BUFFER_SIZE);
    void setLocalName(const char* localName); // max length 29
    void setName(const char* localName); // max length 29 same as setLocalName
    void begin();
    void poll();
    void setConnectedHandler(void(*disconnectHandler)(BLECentral& central));
    void setDisconnectedHandler(void(*disconnectHandler)(BLECentral& central));
    size_t write(uint8_t);
    size_t write(const uint8_t*, size_t);
    int read();
    int available();
    void flush();
    int peek();
    void close();
    bool isConnected();
    void setDebugStream(Print* out);
    size_t bytesToBeSent(); // bytes in buffer to be sent
    void clearTxBuffer();
    static void bleWriteAfterDelay();


    // NOTE: count is number of 1.25mS to delay between send blocks of BLE data
    // this number should be > maxConnection interval number so previous block is picked up prior to sending next one.
    // must be call BEFORE begin() is called, otherwise ignored
    // sets sendDelay to match (maximumConnectionInterval * 1.25mS) + 1mS
    // note: minimumConnectionInterval,maximumConnectionInterval are actually counts of 1.25mS each
    // i.e. 80 => 100mS timeInterval
    void setConnectionInterval(unsigned short minimumConnectionInterval_mS, unsigned short maximumConnectionInterval_mS);

  protected:
    void internalBleWriteAfterDelay();
    Print* debugOut;
    const char* _localName;
    void init(size_t bufferSize );

  private:
    void setSendDelay(unsigned short maximumConnectionInterval);
    bool connectCalled;
    bool timerRunning = false;

    void setTxBuffer(size_t txBufferSize);
    size_t txBufferSize;
    size_t txHead;
    size_t txTail;
    uint8_t* txBuffer;
    static const size_t DEFAULT_SEND_BUFFER_SIZE = 1024; // Max data size pfodApp msg
    static const size_t defaultBufferSize = 32; // > BLE msg size
    uint8_t defaultBuffer[defaultBufferSize]; // if malloc fails

    unsigned long sendDelay_mS;
    uint8_t sendBlock[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];

    uint32_t startTimer();
    uint32_t stopTimer();
    lp_timer lp_BLEBufferedSerial_timer;

    static const int BLE_RX_MAX_LENGTH = 256;
    static volatile size_t rxHead;
    static volatile size_t rxTail;
    volatile static uint8_t rxBuffer[BLE_RX_MAX_LENGTH];
    volatile static bool connected;

    static void connectHandler(BLECentral& central);
    static void disconnectHandler(BLECentral& central);
    static void receiveHandler(BLECentral& central, BLECharacteristic& rxCharacteristic);
    void addReceiveBytes(const uint8_t* bytes, size_t len);
    void(*_connectHandler)(BLECentral& central);  // extra handler to call on connect NULL if not set
    void(*_disconnectHandler)(BLECentral& central); // extra handler to call on disconnect NULL if not set

    // ====  create Nordic UART service =========
    BLEService _uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor _uartNameDescriptor = BLEDescriptor("2901", "UART");
    BLECharacteristic _rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH); // == TX on central (android app)
    BLEDescriptor _rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Write)");
    BLECharacteristic _txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH); // == RX on central (android app)
    BLEDescriptor _txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Notify)");

};


// =========== end lp_BLESerial definitions


#endif
