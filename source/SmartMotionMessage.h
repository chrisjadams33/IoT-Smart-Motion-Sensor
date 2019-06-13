#include "mbed.h"

class SmartMotionMessage {
public:
    // constructor
    SmartMotionMessage(short int senderAddress, short int targetAddress, uint8_t p, uint8_t command, int data);
    
    enum sm_command_t {
        provision,
        query,
        update,
        warning,
        breakin,
        zero
    };
    void setSenderAddress(short int senderAddress);
    void setData(int data);
    void print();
    const uint8_t* serviceDataAdvPacket();
    
    uint8_t priority();
    short int targetAddr();
    uint8_t cmd();
    int data();
    uint8_t id();
    
    // custom queue comparator (High priority at top)
    struct PriorityComparator
    {
      bool
      operator()(SmartMotionMessage & obj1, SmartMotionMessage & obj2) const
      {
        // return true if o1 is ordered before p2
        if (obj1.priority() < obj2.priority())
          return true;
        return false;
      }
    };
    
    // Custom Hash Functor
    struct Hasher
    {
        size_t
        operator()(SmartMotionMessage obj) const
        {
            return ( obj.data() + ( (size_t)obj.targetAddr()<< 16 ) + ( ((size_t)obj.priority())<< 8 ) +  obj.cmd() );
        }
    };
    
    // custom hash comparator
    struct Comparator
    {
      bool
      operator()(SmartMotionMessage m1, SmartMotionMessage m2) const
      {
        if( m1.cmd() != m2.cmd())
            return false;
        if( m1.targetAddr() != m2.targetAddr() )
            return false;
        if( m1.data() != m2.data() )
            return false;
        return true;
      }
    };


private:
    short int smTargetAddr;
    short int smSenderAddr;
    uint8_t smPriority;
    sm_command_t smCmd;
    int smData;
    uint8_t smID;
};

