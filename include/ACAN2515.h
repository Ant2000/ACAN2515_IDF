//----------------------------------------------------------------------------------------
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//----------------------------------------------------------------------------------------

#pragma once

//----------------------------------------------------------------------------------------

#include <ACAN2515_Buffer16.h>
#include <ACAN2515Settings.h>
#include <MCP2515ReceiveFilters.h>
#include <esp_timer.h>

//----------------------------------------------------------------------------------------

class ACAN2515 {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Constructor: using hardware SPI
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    ACAN2515(const spi_device_handle_t &device, // Hardware SPI object
             gpio_num_t inINT); // INT output of MCP2515


    void setDeviceHandle(const spi_device_handle_t &device);
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Initialisation: returns 0 if ok, otherwise see error codes below
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t begin(const ACAN2515Settings& inSettings,
                   gpio_isr_t inInterruptServiceRoutine);

    uint16_t begin(const ACAN2515Settings& inSettings,
                   gpio_isr_t inInterruptServiceRoutine,
                   ACAN2515Mask inRXM0,
                   const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                   uint8_t inAcceptanceFilterCount);

    uint16_t begin(const ACAN2515Settings& inSettings,
                   gpio_isr_t inInterruptServiceRoutine,
                   ACAN2515Mask inRXM0,
                   ACAN2515Mask inRXM1,
                   const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                   uint8_t inAcceptanceFilterCount);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Error codes returned by begin
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    static constexpr uint16_t kNoMCP2515 = 1 << 0;
    static constexpr uint16_t kTooFarFromDesiredBitRate = 1 << 1;
    static constexpr uint16_t kInconsistentBitRateSettings = 1 << 2;
    static constexpr uint16_t kINTPinIsNotAnInterrupt = 1 << 3;
    static constexpr uint16_t kISRIsNull = 1 << 4;
    static constexpr uint16_t kRequestedModeTimeOut = 1 << 5;
    static constexpr uint16_t kAcceptanceFilterArrayIsNULL = 1 << 6;
    static constexpr uint16_t kOneFilterMaskRequiresOneOrTwoAcceptanceFilters = 1 << 7;
    static constexpr uint16_t kTwoFilterMasksRequireThreeToSixAcceptanceFilters = 1 << 8;
    static constexpr uint16_t kCannotAllocateReceiveBuffer = 1 << 9;
    static constexpr uint16_t kCannotAllocateTransmitBuffer0 = 1 << 10;
    static constexpr uint16_t kCannotAllocateTransmitBuffer1 = 1 << 11;
    static constexpr uint16_t kCannotAllocateTransmitBuffer2 = 1 << 12;
    static constexpr uint32_t kISRNotNullAndNoIntPin = 1 << 13;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Change Mode on the fly
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t changeModeOnTheFly(ACAN2515Settings::RequestedMode inRequestedMode);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Set filters on the fly
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t setFiltersOnTheFly();

    uint16_t setFiltersOnTheFly(ACAN2515Mask inRXM0,
                                const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                uint8_t inAcceptanceFilterCount);

    uint16_t setFiltersOnTheFly(ACAN2515Mask inRXM0,
                                ACAN2515Mask inRXM1,
                                const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                uint8_t inAcceptanceFilterCount);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    end
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    void end();

    uint16_t resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine);

    uint16_t resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         ACAN2515Mask inRXM0,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         uint8_t inAcceptanceFilterCount);

    uint16_t ACAN2515::resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         ACAN2515Mask inRXM0,
                         ACAN2515Mask inRXM1,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         uint8_t inAcceptanceFilterCount);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Receiving messages
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool available();

    bool receive(CANMessage& outFrame);

    typedef void (*tFilterMatchCallBack)(uint8_t inFilterIndex);

    bool dispatchReceivedMessage(tFilterMatchCallBack inFilterMatchCallBack = NULL);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Handling messages to send and receiving messages
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    void isr() const;

    bool isr_core();

private:
    void handleTXBInterrupt(uint8_t inTXB);

    void handleRXBInterrupt();


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Properties
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    spi_device_handle_t mDevice;
    // const SPISettings mSPISettings;
    gpio_num_t mINT;
    bool mRolloverEnable;
    bool driverInitialised = false;
    bool driverPaused = false;

public:
    SemaphoreHandle_t mISRSemaphore;
    SemaphoreHandle_t mAccessSemaphore;

private:
    gpio_isr_t mInterruptServiceRoutine = nullptr;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Receive buffer
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ACAN2515_Buffer16 mReceiveBuffer;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Receive buffer size
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    uint16_t receiveBufferSize() const {
        return mReceiveBuffer.size();
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Receive buffer count
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t receiveBufferCount() const {
        return mReceiveBuffer.count();
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Receive buffer peak count
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t receiveBufferPeakCount() const {
        return mReceiveBuffer.peakCount();
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Call back function array
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    ACANCallBackRoutine mCallBackFunctionArray[6];


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Transmitting messages
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    bool sendBufferNotFullForIndex(uint32_t inIndex) const; // 0 ... 2

    bool tryToSend(const CANMessage& inMessage);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Driver transmit buffer
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    ACAN2515_Buffer16 mTransmitBuffer[3];

    bool mTXBIsFree[3];

public:
    uint16_t transmitBufferSize(const uint8_t inIndex) const {
        return mTransmitBuffer[inIndex].size();
    }

    uint16_t transmitBufferCount(const uint8_t inIndex) const {
        return mTransmitBuffer[inIndex].count();
    }

    uint16_t transmitBufferPeakCount(const uint8_t inIndex) const {
        return mTransmitBuffer[inIndex].peakCount();
    }

private:
    void internalSendMessage(const CANMessage& inFrame, uint8_t inTXB) const;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Polling
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    void poll() const;
    bool paused() const { return driverPaused; }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Private methods
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    uint16_t beginWithoutFilterCheck(const ACAN2515Settings& inSettings,
                                     gpio_isr_t inInterruptServiceRoutine,
                                     ACAN2515Mask inRXM0,
                                     ACAN2515Mask inRXM1,
                                     const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                     uint8_t inAcceptanceFilterCount);

    uint16_t internalBeginOperation(const ACAN2515Settings& inSettings,
                                    ACAN2515Mask inRXM0,
                                    ACAN2515Mask inRXM1,
                                    const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                    uint8_t inAcceptanceFilterCount);

    void write2515Command(uint8_t data) const;

    void write2515Register(uint8_t inRegister, uint8_t inValue) const;

    uint8_t read2515Register(uint8_t inRegister) const;

    uint8_t read2515Status() const;

    uint8_t read2515RxStatus() const;

    void bitModify2515Register(uint8_t inRegister, uint8_t inMask, uint8_t inData) const;

    void setupMaskRegister(ACAN2515Mask inMask, uint8_t inRegister) const;

    uint16_t setRequestedMode(uint8_t inCANControlRegister);

    uint16_t internalSetFiltersOnTheFly(ACAN2515Mask inRXM0,
                                        ACAN2515Mask inRXM1,
                                        const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                        uint8_t inAcceptanceFilterCount);

public:
    void attachMCP2515InterruptPin() const;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    MCP2515 controller state
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint8_t receiveErrorCounter() const;
    uint8_t transmitErrorCounter() const;
    uint8_t errorFlagRegister() const;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    No Copy
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    ACAN2515(const ACAN2515&) = delete ;
    ACAN2515& operator =(const ACAN2515&) = delete ;
    static uint32_t millis() { return esp_timer_get_time() / 1000; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
};

//----------------------------------------------------------------------------------------
