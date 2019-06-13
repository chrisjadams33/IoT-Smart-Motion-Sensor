#include "mbed.h"
#include "lp.h"
#include "mxc_config.h"
#include "lp.h"
#include "gpio.h"
#include "rtc.h"

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ButtonService.h"
#include <events/mbed_events.h>
#include <iostream>
#include <mbed.h>
#include <queue>
#include "unordered_set.h"

#include "max32630fthr.h"
#include "VL53L1X.h"
#include "BMI160.h"
#include "data_collection.h"
#include "SmartMotionMessage.h"

MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);

//Define dev board pins
#define PIN_RX2  P2_0
#define PIN_TX2  P2_1
#define PIN_SDA P3_4
#define PIN_SCL P3_5
#define PIN_SDA2 P5_7
#define PIN_SCL2 P6_0
#define PIN_LED_R P2_4
#define PIN_LED_G P2_5
#define PIN_LED_B P2_6
#define BMI160_I2C_ADDRESS 0x68
#define SHUTDOWN_PIN P3_3

const short int sAddress = 0x00;

// Hardware serial port over DAPLink
Serial daplink(PIN_TX2, PIN_RX2, "daplink") ;

// LEDs
DigitalOut rLED(LED1, 1);
DigitalOut gLED(LED2, 1);
DigitalOut bLED(LED3, 1);

// BUTTON INPUT: FOR PROVISIONING UPON POWER UP
InterruptIn button(P2_3);
 
SmartMotionMessage warning_message(sAddress, 0x7FFF, 4, 4, 0x0000);
SmartMotionMessage breakin_message(sAddress, 0x7FFF, 5, 5, 0x0000);

//static sensor variables
static float baselineA;
static uint16_t baselineDist;
static uint16_t currentDist;
static uint16_t change;
static int armedFlag = -1;     //-1 for disarmed, >=0 for armed
static bool queryReady = true;
static bool updateReady = true;
static bool zeroReady = true;
static bool breakinFlag = true;


//user defined input threshold
static uint16_t userDist = 0x8000;

//acc threshold for waking up ToF sensor
const float delt = 0.03;
const float shattered = 0.15;

//------------------------------------------------------------------------------

//Semaphore adverSlots(5);
//Semaphore calcs(1);

void endTransactionCallback(SmartMotionMessage smMessage);

// declare for endTransactionCallback and checkStatus
void processMessage(SmartMotionMessage);

// declare for arm
void checkStatus(void);

// EVENT QUEUE
static EventQueue eventQueue(/* event count */ 20 * EVENTS_EVENT_SIZE);
EventQueue sensorQueue;
EventQueue repeatQueue;
EventQueue processQueue;

// BLE PARAMETERS
const static char     DEVICE_NAME[] = "SMWS";
const static uint8_t  AUTHENTICATION_CODE[] = {0x53,0x4D,0x57,0x53}; // DEFAULT = "SMWS" BEFORE PROVISIONING
static short int      UNICODE_ADDRESS = 0;
const static char LED_UPDATE[] = {0xA0,0x00};
const static uint8_t SERVICE_DATA_LEN = 14;

// PRIORITY QUEUE: Higher priority messages at top (max priority)
const static size_t MAX_SIZE = 20;
const static size_t MAX_BUCKETS = 30;
priority_queue<SmartMotionMessage, std::vector<SmartMotionMessage>, SmartMotionMessage::PriorityComparator> pq;
etl::unordered_set<SmartMotionMessage,MAX_SIZE,MAX_BUCKETS,SmartMotionMessage::Hasher,SmartMotionMessage::Comparator> setOfSmartMotionMessages;
int endTransactionEvent[] = {-1,-1};

static bool continueScan = true;

gpio_cfg_t gpioLP0;

void toggleLP1(void) {
        pegasus.max14690.ldo2SetMode(MAX14690::LDO_DISABLED);
        pegasus.max14690.ldo3SetMode(MAX14690::LDO_DISABLED);
        //Clear existing wake-up config
        LP_ClearWakeUpConfig();
        //Clear any event flags
        LP_ClearWakeUpFlags();
        //configure wake-up on GPIO
        LP_ConfigGPIOWakeUpDetect(&gpioLP0, 0, LP_WEAK_PULL_UP);
        //LP_ConfigRTCWakeUp(unsigned int comp0_en, unsigned int comp1_en, unsigned int prescale_cmp_en, unsigned int rollover_en);
        // set up RTC for wake up
        uint32_t compareCount[RTC_NUM_COMPARE] = {5,0};
        rtc_cfg_t rtcCfg;
        rtcCfg.prescaler = RTC_PRESCALE_DIV_2_11;
        rtcCfg.prescalerMask = RTC_PRESCALE_DIV_2_11;
        rtcCfg.compareCount[0] = 1;
        rtcCfg.compareCount[1] = 0;
        rtcCfg.snoozeCount = 0;
        rtcCfg.snoozeMode = RTC_SNOOZE_DISABLE;
        RTC_Init(&rtcCfg);
        RTC_ClearFlags( RTC_FLAGS_CLEAR_ALL);
        RTC_Start();
        LP_ConfigRTCWakeUp(1, 0, 0, 0);
        //interrupts are disabled in LP_EnterLP0
        LP_EnterLP1();
        //firmware will reset with no prior knowledge on wake-up
        // resume after waking up
        pegasus.max14690.ldo2SetMode(MAX14690::LDO_ENABLED);
        pegasus.max14690.ldo3SetMode(MAX14690::LDO_ENABLED);
        // go back to sleep after processing data
        // not sure if we should tie that to an event, or just have it go back to sleep after x seconds
}

//-----------------------SENSOR CODE--------------------------------------------
//------------------------------------------------------------------------------
data_collection sensors(PIN_SDA, PIN_SCL, SHUTDOWN_PIN, PIN_SDA2, PIN_SCL2, BMI160_I2C_ADDRESS);

//disarm system
void disarm(void){
    if (armedFlag > -1)
        //end call_every using armedFlag
        sensorQueue.cancel(armedFlag);
    armedFlag = -1;
    userDist = 0x8000;
    printf("disarmed\n\r");
}


//send alert upon warning or breakin event
void send_alert(SmartMotionMessage message){
    if(setOfSmartMotionMessages.find(message) == setOfSmartMotionMessages.end()) {
        // If not in hashmap, add to hashmap and enqueue
        setOfSmartMotionMessages.insert(message); // Add to hashmap
        pq.push(message);                    // Enqueue
        
        message.print();
        // Check if message is at top of priority queue
        if(SmartMotionMessage::Comparator()(message,pq.top())) {
            // Process message
            processMessage(message);
        }
    }
}

void checkDistance(void){
    currentDist = sensors.check_ToF();              //check ToF Sensor
    change = abs(currentDist - baselineDist);       //update change in dist
    //breakin event
    if (change > userDist){     //if broke dist threshold
        sensorQueue.call(Callback<void(void)>(disarm));   //disarm
        breakin_message.setData(change);
        send_alert(breakin_message);        //send message
    }
    breakinFlag = true;     //nolonger checking ToF sensor
}
void checkStatus(void){
    float a_sum = baselineA;
    if (breakinFlag)       //if not currently checking ToF sensor
        a_sum = sensors.check_accelerometer(); //check acceleormeter
    //if not currently checking ToF sensor
    if (breakinFlag){
        if (a_sum > baselineA + delt){
            //broke acc threshold so check ToF sensor
            sensorQueue.call_in(1000, Callback<void(void)>(checkDistance));
            breakinFlag = false;    //checking ToF sensor
        }
        if (a_sum < baselineA - delt){
            //broke acc threshold so check ToF sensor
            sensorQueue.call_in(1000, Callback<void(void)>(checkDistance));
            breakinFlag = false;    //checking ToF sensor
        }
    }
    if (a_sum > (baselineA + shattered) || a_sum < (baselineA - shattered)){
        //detected high force, send warning message
        send_alert(warning_message);
    }
}
//arm system to check accelerometer every 0.5s
void arm(void){
    printf("Armed\n\r");
    armedFlag = sensorQueue.call_every(500, checkStatus);
    //printf("Armed Flag = %u\n\r", armedFlag);
}

//zero sensors
void zero(void){
    printf("Zeroing...\n\r");
    baselineA = sensors.BMI_baseline();
    printf("Accelerometer Baseline: %.2f\n\r", baselineA);
    
    baselineDist = sensors.ToF_baseline();
    currentDist = baselineDist;
    printf("ToF Baseline: %u\n\r", baselineDist);
}

//------------------------------------------------------------------------------
//----------------------------BLUETOOTH CODE------------------------------------
void queryAgain(void){
    queryReady = true;
}
void updateAgain(void){
    updateReady = true;
} 
void zeroAgain(void){
    zeroReady = true;
}

void printMacAddress()
{
    /* Print out device MAC address to the console*/
    Gap::AddressType_t addr_type;
    Gap::Address_t address;
    BLE::Instance().gap().getAddress(&addr_type, address);
    printf("DEVICE MAC ADDRESS: ");
    for (int i = 5; i >= 1; i--){
        printf("%02x:", address[i]);
    }
    printf("%02x\r\n", address[0]);
}

// and Bake
void andBake(void)
{bLED=(bLED&1)^1;}

// Shake
void shake(void)
{
    bLED=(bLED&1)^1;
    eventQueue.call_in(1000,Callback<void(void)>(andBake));
}

// Release button to stop advertising (Provisioning)
void stopAdvertisingCallback(void){
    BLE::Instance().gap().stopAdvertising();
}

// endTransaction: remove executed message from queue, hashmap, process next message in queue
void endTransactionCallback(void) {
    // Stop advertising (but not all transactions start advertising)
    ble_error_t error = BLE::Instance().gap().stopAdvertising();
    if(error){
        //printf("ERROR: stopAdvertising()\n\r");
    }
    // Pop top of queue
    SmartMotionMessage oldMessage = pq.top();
    pq.pop();
    // Remove rom hashmap
    setOfSmartMotionMessages.erase(oldMessage);
    //oldMessage.print();
    endTransactionEvent[0] = -1;
    // Process next message in queue
    // Check if priority in sync
    if(pq.empty())
        return;
    else if(!SmartMotionMessage::Comparator()(oldMessage,pq.top())) {
        SmartMotionMessage nextMessage = pq.top();
        processQueue.call(Callback<void(SmartMotionMessage)>(processMessage),nextMessage);
    }
}

// scheduleEndTransactionEvent
void scheduleEndTransactionEvent(int priority) {
    // No events happening
    if(endTransactionEvent[0] == -1){
        endTransactionEvent[0] = eventQueue.call_in(10000, Callback<void(void)>(endTransactionCallback));
        endTransactionEvent[1] = priority;
    }
    // Event of lower priority happening
    else if(priority > endTransactionEvent[1]) {
        // cancel current transaction, will finish later
        eventQueue.cancel(endTransactionEvent[0]);
        // update endTransactionEvent
        endTransactionEvent[0] = eventQueue.call_in(10000, Callback<void(void)>(endTransactionCallback));
        endTransactionEvent[1] = priority;
    }
    else if(priority == endTransactionEvent[1]){
        //printf("Equal priority event...\n\r");
    }
}

// Relay message for 10 seconds (10 seconds specified in processMessage)
void relayMessage(const uint8_t* data) {
    // Create service data packet
    uint8_t serviceData[14];
    std::copy(AUTHENTICATION_CODE,AUTHENTICATION_CODE+4,serviceData);
    std::copy(data,data+11,serviceData+4);
    delete [] data; // delete to free memory
    
    /* setup advertising */
    BLE &ble = BLE::Instance();
    ble.gap().clearAdvertisingPayload();
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);  // BLE only
    ble.gap().accumulateAdvertisingPayload(         // Complete Local Name = "SMWR"
        GapAdvertisingData::COMPLETE_LOCAL_NAME,
        (uint8_t *)DEVICE_NAME,                     
        sizeof(DEVICE_NAME)-1
    );
    // set payload
    ble.gap().accumulateAdvertisingPayload(         // SERVICE DATA = data
        GapAdvertisingData::SERVICE_DATA,
        serviceData,
        SERVICE_DATA_LEN);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED); // Not Scan Requestable
    ble.gap().setAdvertisingInterval(35); // Min Adv Interval = 35
    ble.gap().startAdvertising();
}

// Query request
void queryRequest() {
    /* setup advertising */
    BLE &ble = BLE::Instance();
    ble.gap().clearAdvertisingPayload();
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);  // BLE only
    ble.gap().accumulateAdvertisingPayload(         // Complete Local Name = "SMWR"
        GapAdvertisingData::COMPLETE_LOCAL_NAME,
        (uint8_t *)DEVICE_NAME,                     
        sizeof(DEVICE_NAME)-1
    );
    // Smart Motion Message
    const uint8_t data[] = {
        (UNICODE_ADDRESS & 0xFF00)>>8,
        (UNICODE_ADDRESS & 0xFF),
        (uint8_t)0x7F,(uint8_t)0xFF,
        (uint8_t)2,
        (uint8_t)2,
        (userDist & 0xFF00)>>8,(userDist & 0xFF),
        (change & 0xFF00)>>8,(change & 0xFF)
    };
    // Crerate service data packet
    uint8_t serviceData[14];
    std::copy(AUTHENTICATION_CODE,AUTHENTICATION_CODE+5,serviceData);
    std::copy(data,data+11,serviceData+4);
    
    // set payload
    ble.gap().accumulateAdvertisingPayload(         // SERVICE DATA = data
        GapAdvertisingData::SERVICE_DATA,
        serviceData,
        SERVICE_DATA_LEN);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED); // Not Scan Requestable
    ble.gap().setAdvertisingInterval(35); // Min Adv Interval = 35
    ble.gap().startAdvertising();
}

//------------------------------------------------------------------------------
// Process Message at top of priority queue
void processMessage(SmartMotionMessage smMessage) {
    ////printf("Process Message\n\r");
    int msgCmd = smMessage.cmd();
    // If not for this device, relay message
    if(smMessage.targetAddr() != UNICODE_ADDRESS)
        msgCmd = 100;
    
    continueScan = false;
        
    switch(msgCmd) {
        // Provision
        case 1 :
            printf("Provisioning\n\r");
            UNICODE_ADDRESS = (short int)smMessage.data();
            warning_message.setSenderAddress(UNICODE_ADDRESS);
            breakin_message.setSenderAddress(UNICODE_ADDRESS);
            break;
        //Query
        case 2 :
            if (!queryReady)
                break;
            queryReady = false;
            printf("Query request\n\r");
            processQueue.call(Callback<void(void)>(queryRequest));
            repeatQueue.call_in(10000, Callback<void(void)> (queryAgain));
            break;
        // Update
        case 3 :
            if (!updateReady)
                break;
            updateReady = false;    
            printf("Update: 0x%X\n\r",smMessage.data());
            //printf("Shifted: 0x%X\n\r",(smMessage.data()>>31));
            //printf("Armed Flag: %u\n\r",armedFlag);
            if (((smMessage.data() >> 31)==0)){
                //Arming system (MSB = 1)
                //bLED = !bLED;
                if (armedFlag == -1)
                    sensorQueue.call(Callback<void(void)>(arm));
                userDist = ((smMessage.data() >> 16 )& 0x0000ffff );
            }
        
            else if ((((smMessage.data() >> 31)&1)==1) && (armedFlag != -1)){
                //disarm system
                //bLED = !bLED;
                sensorQueue.call(Callback<void(void)>(disarm));
            }
            repeatQueue.call_in(10000, Callback<void(void)> (updateAgain));
            break;
        //Zero
        case 6 :
            if (!zeroReady)
                break;
            zeroReady = false;
            sensorQueue.call(Callback<void(void)>(zero));
            repeatQueue.call_in(10000, Callback<void(void)> (zeroAgain));
            break;
        // Message not for this device, relay message
        default :
            printf("Relaying Message\n\r");
            processQueue.call(Callback<void(const uint8_t*)>(relayMessage),smMessage.serviceDataAdvPacket());
    }
    
    continueScan = true;
    scheduleEndTransactionEvent(smMessage.priority());
}

void parseAdvertisement(const uint8_t* value) {
    // First authentication check (Packet structure):
    // 1. Byte 0 = 0x05 (Length of complete local name)
    // 2. Byte 1 = 0x09 (Complete local name)
    // 3. Byte 6 = 0x0D (Length of Service Data)
    // 4. Byte 7 = 0x16 (Service Data)
    // 5. Complete local name == "SMWS"
    ////printf("0x%X%X %X%X ",value[0],value[1],value[6],value[7]);
    if(value[0] == 0x2 && value[1] == 0x1 && value[2] == 0x6)
        value += 3;
    if(value[0] != 0x5 || value[1] != 0x9 || value[6] != 0xF || value[7] != 0x16 ||
            memcmp(value + 2, DEVICE_NAME, 4) != 0) {
        return;
    }

    
    // AES decrypt bytes 8-19
    
    // 2nd authentication check (User define passcode):
    // 6. Authentication Code == User specified ("SMWS" on Power Up)
    if(memcmp(value + 8, AUTHENTICATION_CODE, 4) != 0) {
        //printf("Authentication Code Failure\n\r");
        return;
    }
    //std::cout<<("Parse SM Message\n\r");
    //std::flush(std::cout);
    // Parse message
    SmartMotionMessage tempMessage = SmartMotionMessage(
        (value[12]<<8)+value[13],                                       // Sender Address (2 bytes)
        (value[14]<<8)+value[15],                                       // Target Address (2 bytes)
        value[16],                                                      // Priority (1 byte)
        value[17],                                                      // Command (1 byte)
        (value[18]<<24) + (value[19]<<16) + (value[20]<<8) + value[21]  // Data (4 bytes)
    );                                                                  // Message ID (1 byte)
    //tempMessage.setID((int)pq.size());
    //tempMessage.print();
    
    // Enqueue message
    // Check if message in hashmap
    if(setOfSmartMotionMessages.find(tempMessage) == setOfSmartMotionMessages.end()) {
        // If not in hashmap, add to hashmap and enqueue
        setOfSmartMotionMessages.insert(tempMessage);       // Add to hashmap
        pq.push(tempMessage);                               // Enqueue
        
        // Check if message is at top of priority queue
        if(SmartMotionMessage::Comparator()(tempMessage,pq.top())) {
            // Process message
            processMessage(tempMessage);
        }
    }
    // Message already in queue/hashmap
    //std::cout<<("End parseAdvertisement\n\r");
    //std::flush(std::cout);
}

// supposedly should fix crashing issues... I'm not convinced yet
void advertisementCallback(const Gap::AdvertisementCallbackParams_t *params) {
    // callback data pointer
    const uint8_t* value = params->advertisingData;
    if(continueScan == true)
        eventQueue.call(Callback<void(const uint8_t*)>(parseAdvertisement),value);
}

// define for startScanCallback
void stopScanCallback(void);

// Scan for 200 ms
void startScanCallback(void) {
    BLE &ble = BLE::Instance();
    /* setup scanning */
    ble.gap().setScanParams(
        GapScanningParams::SCAN_INTERVAL_MIN, // interval between two scan window in ms
        GapScanningParams::SCAN_WINDOW_MIN,   // scan window: period during which the device listen for advertising packets
        0,                                    // the scan process never ends
        false                                 // no scan request
    );
    if (continueScan == true)
        ble.startScan(advertisementCallback);
    // stop scanning in 200 ms
    eventQueue.call_in(200,Callback<void(void)>(stopScanCallback));
}

// Stop scanning for 5 seconds
void stopScanCallback(void) {
    BLE::Instance().gap().stopScan();

    eventQueue.call_in(5000,Callback<void(void)>(startScanCallback));
}

// Press button to start advertising for provisioning
void buttonPressedCallback(void)
{
    //printf("Button Pressed: Advertise for Provisioning\n\r");
    /* setup advertising */
    BLE &ble = BLE::Instance();
    ble.gap().clearAdvertisingPayload();
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);  // BLE only
    ble.gap().accumulateAdvertisingPayload(         // Complete Local Name = "SMWR"
        GapAdvertisingData::COMPLETE_LOCAL_NAME,
        (uint8_t *)DEVICE_NAME,                     
        sizeof(DEVICE_NAME)-1
    ); 
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED); // Not Scan Requestable
    ble.gap().setAdvertisingInterval(100);//GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX); // Max Adv Interval = 0x4000
    ble.gap().startAdvertising();
    
    printMacAddress();
    
    eventQueue.call(Callback<void(void)>(startScanCallback));    
}

// LED blink callback
void blinkCallback(void)
{rLED = !rLED; /* Do blinky on LED1 to indicate system aliveness. */}
// BLE init error
void onBleInitError(BLE &ble, ble_error_t error){
    //printf("BLE failed to initialize!\n\r");/* Initialization error handling should go here */
}

//------------------------------------------------------------------------------
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    // init button callbacks
    button.fall(buttonPressedCallback);
    button.rise(stopAdvertisingCallback);
}

//------------------------------------------------------------------------------
void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int main()
{
    //rLED = 0;
    bLED = 1;
    gLED = 1;
    
    wait(1);
    ////printf("Done waiting\n\r");
    while(!sensors.setup());
    printf("setup complete\n\r");
    
    //baselineA = sensors.BMI_baseline();
//    printf("Accelerometer Baseline: %.2f\n\r", baselineA);
//    
//    baselineDist = sensors.ToF_baseline();
//    currentDist = baselineDist;
//    printf("ToF Baseline: %u\n\r", baselineDist);
    
    Thread sensorThread(osPriorityLow);
    sensorThread.start(callback(&sensorQueue, &EventQueue::dispatch_forever));
    
    Thread repeatThread(osPriorityLow);
    repeatThread.start(callback(&repeatQueue, &EventQueue::dispatch_forever));
    
//BLUETOOTH SHIT

    //Thread eventThread(osPriorityNormal);
    //eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
    
    Thread processThread(osPriorityHigh);
    processThread.start(callback(&processQueue, &EventQueue::dispatch_forever));
    
    //eventQueue.call_every(1000, blinkCallback);
    printf("Ready to Provision\n\r");

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);
    
    shake();
    eventQueue.call(Callback<void(void)>(startScanCallback));
    eventQueue.dispatch_forever();
    
    
    //eventQueue.call(toggleLP1);
    
    
}