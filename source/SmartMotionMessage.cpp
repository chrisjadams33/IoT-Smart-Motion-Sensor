#include "SmartMotionMessage.h"
#include "mbed.h"

SmartMotionMessage::SmartMotionMessage(short int senderAddress, short int targetAddress, uint8_t p, uint8_t command, int data) {
    smSenderAddr = senderAddress;
    smTargetAddr = targetAddress;
    smPriority = p;
    smCmd = static_cast<sm_command_t>(command);
    smData = data;
}

void SmartMotionMessage::setSenderAddress(short int senderAddress){smSenderAddr = senderAddress;}
void SmartMotionMessage::setData(int data){smData = data;}

void SmartMotionMessage::print(){
    printf("Smart Motion Message:\n\r");
    printf("    Target Address: 0x%X\n\r",smTargetAddr);
    printf("    Sender Address: 0x%X\n\r",smSenderAddr);
    printf("    Priority: %i\n\r",smPriority);
    printf("    Command: %i\n\r",smCmd);
    printf("    Data: 0x%X\n\r",smData);
    printf("    ID: %i\n\r",smID);
}

const uint8_t* SmartMotionMessage::serviceDataAdvPacket() {
    uint8_t* data = NULL;
    data = new uint8_t[8];
    data[2]=     (uint8_t)((smTargetAddr & 0xFF00) >> 8);
    data[3]=     (uint8_t)(smTargetAddr & 0xFF);
    data[0]=     (uint8_t)((smSenderAddr & 0xFF00) >> 8);
    data[1]=     (uint8_t)(smSenderAddr & 0xFF);
    data[4]=     smPriority;
    data[5]=     smCmd;
    data[6]=     (uint8_t)((smData & 0xFF000000) >> 24);
    data[7]=     (uint8_t)((smData & 0xFF0000) >> 16);
    data[8]=     (uint8_t)((smData & 0xFF00) >> 8);
    data[9]=     (uint8_t)(smData & 0xFF);
    return data;
}

uint8_t SmartMotionMessage::priority(){return smPriority;}
short int SmartMotionMessage::targetAddr(){return smTargetAddr;}
uint8_t SmartMotionMessage::cmd(){return (uint8_t)smCmd;}
int SmartMotionMessage::data(){return smData;}
uint8_t SmartMotionMessage::id(){return smID;}
