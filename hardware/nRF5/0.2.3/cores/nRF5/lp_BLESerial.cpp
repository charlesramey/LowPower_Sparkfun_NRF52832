#include "lp_BLESerial.h"

//#define DEBUG

volatile size_t lp_BLESerial::rxHead = 0;
volatile size_t lp_BLESerial::rxTail = 0;

volatile uint8_t lp_BLESerial::rxBuffer[BLE_RX_MAX_LENGTH];
volatile bool lp_BLESerial::connected = false;

// ========== lp_BLESerial methods

// static method
void lp_BLESerial::bleWriteAfterDelay() {
	    ((lp_BLESerial*)(BLEPeripheral::pollInstance))->internalBleWriteAfterDelay();
}

void lp_BLESerial::internalBleWriteAfterDelay() {
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("bleWriteAfterDelay at "); debugOut->print(millis()); debugOut->println();
  }
#endif

  BLEPeripheral::poll();

  // send next 20bytes here and stop timer if buffer empty
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("bytesToBeSent() :"); debugOut->println(bytesToBeSent());
  }
#endif

  if (!isConnected() || (txHead == txTail)) {  // common cases
    stopTimer();
    return; // nothing to do
  }

  // else have something to send
  // send next 20 bytes
  size_t i = 0; // max to send is
  while ((i < BLE_ATTRIBUTE_MAX_VALUE_LENGTH) && (txHead != txTail)) {
    sendBlock[i] = (const uint8_t)(txBuffer[txTail]);
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print((char)txBuffer[txTail]);
    }
#endif // DEBUG    
    i++;
    txTail = (txTail + 1) % txBufferSize;
  }

  _txCharacteristic.setValue(sendBlock, i);
  BLEPeripheral::poll();
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->println();
  }
#endif // DEBUG    
  if (txHead == txTail) {
    stopTimer();
  }
}

void lp_BLESerial::setDebugStream(Print* out) {
  debugOut = out;
}

void lp_BLESerial::setName(const char* localName) {
  setLocalName(localName);
}

void lp_BLESerial::setLocalName(const char* localName) {
  if (localName == NULL) {
    BLEPeripheral::setLocalName("Nordic BLE UART");
  } else {
    BLEPeripheral::setLocalName(localName);
  }
}

lp_BLESerial::lp_BLESerial(size_t _txBufferSize) : BLEPeripheral() {
  init(_txBufferSize);
}


void lp_BLESerial::setConnectedHandler(void(*connectHandler)(BLECentral& central)) {
  _connectHandler = connectHandler;
}
void lp_BLESerial::setDisconnectedHandler(void(*disconnectHandler)(BLECentral& central)){
  _disconnectHandler = disconnectHandler;
}

void lp_BLESerial::init(size_t _txBufferSize ) {
  _connectHandler = NULL;
  _disconnectHandler = NULL;
  setTxBuffer(_txBufferSize);
  debugOut = NULL;
  setSendDelay(DEFAULT_MAX_CONNECTION_INTERVAL_mS);

  setLocalName(NULL); // use default to start with
  addAttribute(_uartService);
  addAttribute(_uartNameDescriptor);
  setAdvertisedServiceUuid(_uartService.uuid());
  addAttribute(_rxCharacteristic);
  addAttribute(_rxNameDescriptor);
  _rxCharacteristic.setEventHandler(BLEWritten, lp_BLESerial::receiveHandler);
  setEventHandler(BLEConnected, lp_BLESerial::connectHandler);
  setEventHandler(BLEDisconnected, lp_BLESerial::disconnectHandler);
  addAttribute(_txCharacteristic);
  addAttribute(_txNameDescriptor);

  clearTxBuffer();
};

void lp_BLESerial::clearTxBuffer() {
#ifdef DEBUG
  if (debugOut) {
    debugOut->println("lp_BLESerial clearTxBuffer.");
  }
#endif // DEBUG	
  txHead = 0;
  txTail = 0;
  stopTimer();
}

uint32_t lp_BLESerial::startTimer() {
  if (timerRunning) {
    // already running
    return 0; // no error
  }
  // else
  uint32_t err_code;
  // access the global C code method
  err_code = lp_BLEBufferedSerial_timer.startTimer(sendDelay_mS, bleWriteAfterDelay); // timeout in mS 
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->println();
    debugOut->print(" startTimer errorCode:"); debugOut->println(err_code);
  }
#endif
  if (err_code == 0) {
    timerRunning = true;
  }
  return err_code;
}

uint32_t lp_BLESerial::stopTimer() {
  if (!timerRunning) {
    // already stopped
    return 0; // no error
  }
  // else
  uint32_t err_code;
  // access the global C code method
  err_code = lp_BLEBufferedSerial_timer.stop();
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->println();
    debugOut->print("stopTimer errorCode:"); debugOut->println(err_code);
  }
#endif
  if (err_code == 0) {
    timerRunning = false;
  }
  return err_code;
}

void lp_BLESerial::setConnectionInterval(unsigned short minimumConnectionInterval_mS, unsigned short maximumConnectionInterval_mS) {
  unsigned short minConInterval = BLE_GAP_CP_MIN_CONN_INTVL_MIN + (BLE_GAP_CP_MIN_CONN_INTVL_MIN >> 2) + 1; // in 1.25ms units => mS
  unsigned short maxConInterval = BLE_GAP_CP_MAX_CONN_INTVL_MAX + (BLE_GAP_CP_MAX_CONN_INTVL_MAX >> 2); // in 1.25ms units => mS truncated down

#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print(" maxConInterval ");
    debugOut->print(maxConInterval);
    debugOut->print(" minConInterval ");
    debugOut->print(minConInterval);
    debugOut->println();
  }
#endif // DEBUG    

  if (maximumConnectionInterval_mS > maxConInterval) {
    maximumConnectionInterval_mS = maxConInterval; // 4sec max
  }
  if (maximumConnectionInterval_mS < minConInterval) {
    maximumConnectionInterval_mS = minConInterval; // 7.5mS min
  }

  if (minimumConnectionInterval_mS > maxConInterval) {
    minimumConnectionInterval_mS = maxConInterval; // 4sec max
  }
  if (minimumConnectionInterval_mS < minConInterval) {
    minimumConnectionInterval_mS = minConInterval; // 7.5mS min
  }

  if (minimumConnectionInterval_mS > maximumConnectionInterval_mS) {
    minimumConnectionInterval_mS = maximumConnectionInterval_mS;
  }

#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print(" minimumConnectionInterval_mS ");
    debugOut->print(minimumConnectionInterval_mS);
    debugOut->print(" maximumConnectionInterval_mS ");
    debugOut->print(maximumConnectionInterval_mS);
    debugOut->println();
  }
#endif // DEBUG    

  BLEPeripheral::setConnectionInterval(minimumConnectionInterval_mS, maximumConnectionInterval_mS);
  // set sendDelay to maxConnectionInterval
  //It is important to know that maximum number of packets per connection event is dependent on the BLE stack/chipsets and is limited
  // to 4 packets per connection event with iOS, and 6 packets per connection event in Android.
  // for simplicity just send on per connection
  setSendDelay(maximumConnectionInterval_mS);
}

void lp_BLESerial::setSendDelay(unsigned short maximumConnectionInterval_mS) {
  sendDelay_mS = maximumConnectionInterval_mS;
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("setSendDelay to ");
    debugOut->print(sendDelay_mS);
    debugOut->println(" mS");
  }
#endif // DEBUG    
}

void lp_BLESerial::setTxBuffer(size_t _txBufferSize) {
  txHead = 0;
  txTail = 0;

  if ((txBuffer == NULL) && (_txBufferSize > defaultBufferSize)) {
    txBufferSize = _txBufferSize;
    txBuffer = (uint8_t*)malloc(_txBufferSize);
  }
  if (txBuffer == NULL) {
    txBufferSize = defaultBufferSize;
    txBuffer = defaultBuffer;
  }
}


bool lp_BLESerial::isConnected() {
  return (connected && _txCharacteristic.subscribed() && BLEPeripheral::connected());
}

void lp_BLESerial::begin() {
  BLEPeripheral::begin();
  setSendDelay(this->_device->getMaximumConnectionInterval());
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->println(F("lp_BLESerial::begin()"));
  }
#endif
}


void lp_BLESerial::close() {
  connected = false; // stop writes
  BLEPeripheral::disconnect();
}

void lp_BLESerial::poll() {
  BLEPeripheral::poll();
}

size_t lp_BLESerial::write(const uint8_t* bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {
    write(bytes[i]);
  }
  return len; // just assume it is all written
}


size_t lp_BLESerial::write(uint8_t c) {
  BLEPeripheral::poll();
  if (!isConnected()) {
    return 0;
  }

  size_t rtn = 1;
  size_t i = (txHead + 1) % txBufferSize;
  if (i == txTail) {
    // If the output buffer is full, just drop the char
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("buffer full drop byte "); debugOut->println((char)c);
    }
#endif // DEBUG    
    return rtn;
  }

  txBuffer[txHead] = c;
  txHead = i;
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print((char)c);
  }
#endif // DEBUG    

  startTimer();
  return rtn;
}

size_t lp_BLESerial::bytesToBeSent() {
  if (txTail <= txHead) {
    return (txHead - txTail);
  } // else
  return (txHead + txBufferSize - txTail);
}

int lp_BLESerial::read() {
  if (rxTail == rxHead) {
    return -1;
  }
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  rxTail = (rxTail + 1) % sizeof(rxBuffer);
  uint8_t b = rxBuffer[rxTail];
  return b;
}

int lp_BLESerial::available() {
  BLEPeripheral::poll();
  int rtn = ((rxHead + sizeof(rxBuffer)) - rxTail ) % sizeof(rxBuffer);
  //#ifdef DEBUG
  //  if (debugOut != NULL) {
  //    debugOut->print("lp_BLESerial available():"); debugOut->println(rtn);
  //  }
  //#endif // DEBUG
  return rtn;
}

void lp_BLESerial::flush() {
  BLEPeripheral::poll();
}

int lp_BLESerial::peek() {
  BLEPeripheral::poll();
  if (rxTail == rxHead) {
    return -1;
  }
  size_t nextIdx = (rxTail + 1) % sizeof(rxBuffer);
  uint8_t byte = rxBuffer[nextIdx];
  return byte;
}

// called from lp_BLEPeripherial class
void lp_BLESerial::connectHandler(BLECentral& central) {
  connected = true;
  ((lp_BLESerial*)BLEPeripheral::pollInstance)->clearTxBuffer(); // clear anything not sent yet
  if (((lp_BLESerial*)BLEPeripheral::pollInstance)->_connectHandler != NULL) {
    ((lp_BLESerial*)BLEPeripheral::pollInstance)->_connectHandler(central);
  }
}

// called from lp_BLEPeripherial class
void lp_BLESerial::disconnectHandler(BLECentral& central) {
  connected = false;
  ((lp_BLESerial*)BLEPeripheral::pollInstance)->clearTxBuffer(); // clear anything not sent yet
  if (((lp_BLESerial*)BLEPeripheral::pollInstance)->_disconnectHandler != NULL) {
    ((lp_BLESerial*)BLEPeripheral::pollInstance)->_disconnectHandler(central);
  }
}

void lp_BLESerial::addReceiveBytes(const uint8_t* bytes, size_t len) {
  // note increment rxHead before writing
  // so need to increment rxTail before reading
  for (size_t i = 0; i < len; i++) {
    size_t nextIdx = (rxHead + 1) % sizeof(rxBuffer);
    if (nextIdx == rxTail) {
      // If the output buffer is full, just drop the char
#ifdef DEBUG
      if (debugOut != NULL) {
        debugOut->print("buffer full drop received byte "); debugOut->print((char)bytes[i]);
      }
#endif // DEBUG    
      return;
    }
    rxHead = nextIdx;
    rxBuffer[rxHead] = bytes[i];
  }
}

void lp_BLESerial::receiveHandler(BLECentral& central, BLECharacteristic& rxCharacteristic) {
  size_t len = rxCharacteristic.valueLength();
  const unsigned char *data = rxCharacteristic.value();
  ((lp_BLESerial*)BLEPeripheral::pollInstance) -> addReceiveBytes((const uint8_t*)data, len);
}
//======================= end lp_BLESerial methods
