//#include <Usb.h>

#define EP_MAXPKTSIZE           64 // max size for data via USB
#define EP_INTERRUPT            0x03
#define HID_REQUEST_GET_REPORT  0x01
#define HID_REQUEST_SET_REPORT  0x09
#define PRIME_MAX_ENDPOINTS     2

#define PRIME_CONTROL_PIPE        0
#define PRIME_OUTPUT_PIPE         1
#define PRIME_INPUT_PIPE          2
#define PRIME_REPORT_BUFFER_SIZE  64

#define PRIME_VID 1008
#define PRIME_PID 1089
#define DEBUG_USB_HOST

class PrimeUsb:USBDeviceConfig
{
public:
  PrimeUsb (USB*);
  virtual uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed);

  virtual uint8_t Release();
  virtual uint8_t Poll();
  virtual uint8_t GetAddress() { return bAddress; };
  void attachOnInit(void (*funcOnInit)(void)) { pFuncOnInit = funcOnInit; };
  bool PrimeConnected;

protected:
  USB *pUsb;
  uint8_t bAddress;
  EpInfo epInfo[PRIME_MAX_ENDPOINTS];

private:
  void onInit();
  void (*pFuncOnInit)(void); // Pointer to function called in onInit()

  bool bPollEnable;
  uint32_t timer;

  uint8_t readBuf[EP_MAXPKTSIZE]; // General purpose buffer for input data
  uint8_t writeBuf[EP_MAXPKTSIZE]; // General purpose buffer for output data

  void readReport(); // read incoming data
  void printReport(); // print incoming date - Uncomment for debugging
};

PrimeUsb::PrimeUsb (USB *p):pUsb(p)
{
  for(uint8_t i = 0; i < PRIME_MAX_ENDPOINTS; i++) {
    epInfo[i].epAddr = 0;
    epInfo[i].maxPktSize = (i) ? 0 : 8;
    epInfo[i].epAttribs = 0;
    epInfo[i].bmNakPower = (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
  }

  if(pUsb) // register in USB subsystem
    pUsb->RegisterDeviceClass(this); //set devConfig[] entry
}

void PrimeUsb::onInit() {
  /*if(pFuncOnInit)
   pFuncOnInit(); // Call the user function*/
}


/* Performs a cleanup after failed Init() attempt */
uint8_t PrimeUsb::Release() {
  PrimeConnected = false;
  pUsb->GetAddressPool().FreeAddress(bAddress);
  bAddress = 0;
  bPollEnable = false;
  return 0;
}

uint8_t test[EP_MAXPKTSIZE] =
{
  0x00, 0xF2, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x48, 0x00, 0x45, 0x00, 0x4C, 0x00, 0x4C, 0x00, 0x4F,
  0x00, 0x00, 0x00, 0x31, 0x78, 0x5E, 0x27, 0x31, 0x06, 0x10, 0x00, 0x00, 0xF0, 0x95, 0x37, 0x30,
  0x14, 0x91, 0x6D, 0x31, 0x94, 0x90, 0x6D, 0x31, 0x94, 0x91, 0x6D, 0x31, 0xE4, 0x91, 0x6D, 0x31,
  0x00, 0x4E, 0x27, 0x31, 0x74, 0x5E, 0x27, 0x31, 0x78, 0x5E, 0x27, 0x31, 0x00, 0x00, 0x00, 0x00,
};

uint8_t PrimeUsb::Poll() {
  if(!bPollEnable)
    return 0;

  if(PrimeConnected)
  {
    uint16_t BUFFER_SIZE = EP_MAXPKTSIZE;
    pUsb->inTransfer(bAddress, epInfo[ PRIME_INPUT_PIPE ].epAddr, &BUFFER_SIZE, readBuf); // input on endpoint 1
    if(millis() - timer > 100)
    { // Loop 100ms before processing data
      readReport();
      printReport();
    }

    if(millis() - timer > 4000) { // Send at least every 4th second
      Notify(PSTR("\r\nPreparing..."), 0x80);

      pUsb->outTransfer(bAddress, epInfo[ PRIME_OUTPUT_PIPE ].epAddr, EP_MAXPKTSIZE, test);
      timer = millis();
      Notify(PSTR("\r\nSending report..."), 0x80);
    }
  }
}


void PrimeUsb::readReport() {
  //ButtonState = (uint32_t)(readBuf[2] | ((uint16_t)readBuf[3] << 8) | ((uint32_t)readBuf[4] << 16));
  Notify(PSTR("\r\nReading report..."), 0x80);
}

void PrimeUsb::printReport() { 
  for(uint8_t i = 0; i < PRIME_REPORT_BUFFER_SIZE; i++) {
    D_PrintHex<uint8_t > (readBuf[i], 0x80);
    Notify(PSTR(" "), 0x80);
  }
  Notify(PSTR("\r\n"), 0x80);
}

uint8_t PrimeUsb::Init(uint8_t parent, uint8_t port, bool lowspeed) {
  uint8_t buf[sizeof (USB_DEVICE_DESCRIPTOR)];
  USB_DEVICE_DESCRIPTOR * udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);
  uint8_t rcode;
  UsbDevice *p = NULL;
  EpInfo *oldep_ptr = NULL;
  uint16_t PID;
  uint16_t VID;

  // get memory address of USB device address pool
  AddressPool &addrPool = pUsb->GetAddressPool();
#ifdef DEBUG_USB_HOST
  Notify(PSTR("\r\nPrime Init"), 0x80);
#endif

  // check if address has already been assigned to an instance
  if(bAddress) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nAddress in use"), 0x80);
#endif
    return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
  }

  // Get pointer to pseudo device with address 0 assigned
  p = addrPool.GetUsbDevicePtr(0);

  if(!p) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
  }

  if(!p->epinfo) {
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nepinfo is null"), 0x80);
#endif
    return USB_ERROR_EPINFO_IS_NULL;
  }

  // Save old pointer to EP_RECORD of address 0
  oldep_ptr = p->epinfo;

  // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
  p->epinfo = epInfo;
  p->lowspeed = lowspeed;

  // Get device descriptor
  rcode = pUsb->getDevDescr(0, 0, sizeof (USB_DEVICE_DESCRIPTOR), (uint8_t*)buf); // Get device descriptor - addr, ep, nbytes, data
  // Restore p->epinfo
  p->epinfo = oldep_ptr;

  if(rcode)
    goto FailGetDevDescr;

  VID = udd->idVendor;
  PID = udd->idProduct;

  if(VID != PRIME_VID || PID != PRIME_PID)
    goto FailUnknownDevice;

  // We are connected
  PrimeConnected = true;

  // Allocate new address according to device class
  bAddress = addrPool.AllocAddress(parent, false, port);

  if(!bAddress)
    return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

  // Extract Max Packet Size from device descriptor
  epInfo[0].maxPktSize = udd->bMaxPacketSize0;

  // Assign new address to the device
  rcode = pUsb->setAddr(0, 0, bAddress);

  if(rcode) {
    p->lowspeed = false;
    addrPool.FreeAddress(bAddress);
    bAddress = 0;
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nsetAddr: "), 0x80);
    D_PrintHex<uint8_t > (rcode, 0x80);
#endif

    return rcode;
  }
#ifdef EXTRADEBUG
  Notify(PSTR("\r\nAddr: "), 0x80);
  D_PrintHex<uint8_t > (bAddress, 0x80);
#endif
  //delay(300); // Spec says you should wait at least 200ms

  p->lowspeed = false;

  //get pointer to assigned address record
  p = addrPool.GetUsbDevicePtr(bAddress);
  if(!p)
    return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

  p->lowspeed = lowspeed;

  // Assign epInfo to epinfo pointer - only EP0 is known
  rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
  if(rcode)
    goto FailSetDevTblEntry;

  /* Initialize data structures for endpoints of device */
  epInfo[ PRIME_OUTPUT_PIPE ].epAddr = 0x2; // output endpoint
  epInfo[ PRIME_OUTPUT_PIPE ].epAttribs = EP_INTERRUPT;
  epInfo[ PRIME_OUTPUT_PIPE ].bmNakPower = USB_NAK_NOWAIT; // Only poll once for interrupt endpoints
  epInfo[ PRIME_OUTPUT_PIPE ].maxPktSize = EP_MAXPKTSIZE;
  epInfo[ PRIME_OUTPUT_PIPE ].bmSndToggle = 0;
  epInfo[ PRIME_OUTPUT_PIPE ].bmRcvToggle = 0;
  epInfo[ PRIME_INPUT_PIPE ].epAddr = 0x01; // report endpoint
  epInfo[ PRIME_INPUT_PIPE ].epAttribs = EP_INTERRUPT;
  epInfo[ PRIME_INPUT_PIPE ].bmNakPower = USB_NAK_NOWAIT; // Only poll once for interrupt endpoints
  epInfo[ PRIME_INPUT_PIPE ].maxPktSize = EP_MAXPKTSIZE;
  epInfo[ PRIME_INPUT_PIPE ].bmSndToggle = 0;
  epInfo[ PRIME_INPUT_PIPE ].bmRcvToggle = 0;

  rcode = pUsb->setEpInfoEntry(bAddress, 3, epInfo);
  if(rcode)
    goto FailSetDevTblEntry;

  delay(200); //Give time for address change

  rcode = pUsb->setConf(bAddress, epInfo[ PRIME_CONTROL_PIPE ].epAddr, 1);
  if(rcode)
    goto FailSetConfDescr;
  Serial.println("here");
  onInit();

  bPollEnable = true;
  Notify(PSTR("\r\n"), 0x80);
  timer = millis();
  return 0; // Successful configuration

  /* Diagnostic messages */
FailGetDevDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailGetDevDescr();
  goto Fail;
#endif

FailSetDevTblEntry:
#ifdef DEBUG_USB_HOST
  NotifyFailSetDevTblEntry();
  goto Fail;
#endif

FailSetConfDescr:
#ifdef DEBUG_USB_HOST
  NotifyFailSetConfDescr();
#endif
  goto Fail;

FailUnknownDevice:
#ifdef DEBUG_USB_HOST
  NotifyFailUnknownDevice(VID, PID);
#endif
  rcode = USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;

Fail:
#ifdef DEBUG_USB_HOST
  Notify(PSTR("\r\nInit Failed, error code: "), 0x80);
  NotifyFail(rcode);
#endif
  Release();
  return rcode;
}


USB Usb;
PrimeUsb myPrime(&Usb);

void setup()
{
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPrime USB Library Started"));
}

void loop()
{
  Usb.Task();
}
