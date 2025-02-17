//------------------------------------------------------------------------------
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//------------------------------------------------------------------------------

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <ACAN2515.h>

//------------------------------------------------------------------------------
//   MCP2515 COMMANDS
//------------------------------------------------------------------------------

static constexpr uint8_t RESET_COMMAND = 0xC0;
static constexpr uint8_t WRITE_COMMAND = 0x02;
static constexpr uint8_t READ_COMMAND = 0x03;
static constexpr uint8_t BIT_MODIFY_COMMAND = 0x05;
static constexpr uint8_t LOAD_TX_BUFFER_COMMAND = 0x40;
static constexpr uint8_t REQUEST_TO_SEND_COMMAND = 0x80;
static constexpr uint8_t READ_FROM_RXB0SIDH_COMMAND = 0x90;
static constexpr uint8_t READ_FROM_RXB1SIDH_COMMAND = 0x94;
static constexpr uint8_t READ_STATUS_COMMAND = 0xA0;
static constexpr uint8_t RX_STATUS_COMMAND = 0xB0;

//------------------------------------------------------------------------------
//   MCP2515 REGISTERS
//------------------------------------------------------------------------------

static constexpr uint8_t BFPCTRL_REGISTER = 0x0C;
static constexpr uint8_t TXRTSCTRL_REGISTER = 0x0D;
static constexpr uint8_t CANSTAT_REGISTER = 0x0E;
static constexpr uint8_t CANCTRL_REGISTER = 0x0F;
static constexpr uint8_t TEC_REGISTER = 0x1C;
static constexpr uint8_t REC_REGISTER = 0x1D;
static constexpr uint8_t RXM0SIDH_REGISTER = 0x20;
static constexpr uint8_t RXM1SIDH_REGISTER = 0x24;
static constexpr uint8_t CNF3_REGISTER = 0x28;
static constexpr uint8_t CNF2_REGISTER = 0x29;
static constexpr uint8_t CNF1_REGISTER = 0x2A;
static constexpr uint8_t CANINTF_REGISTER = 0x2C;
static constexpr uint8_t EFLG_REGISTER = 0x2D;
static constexpr uint8_t TXB0CTRL_REGISTER = 0x30;
static constexpr uint8_t TXB1CTRL_REGISTER = 0x40;
static constexpr uint8_t TXB2CTRL_REGISTER = 0x50;
static constexpr uint8_t RXB0CTRL_REGISTER = 0x60;
static constexpr uint8_t RXB1CTRL_REGISTER = 0x70;

static const uint8_t RXFSIDH_REGISTER[6] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18};

//------------------------------------------------------------------------------
// Note about ESP32
//------------------------------------------------------------------------------
//
// It appears that Arduino ESP32 interrupts are managed in a completely different way
// from "usual" Arduino:
//   - SPI.usingInterrupt is not implemented;
//   - noInterrupts() and interrupts() are NOPs;
//   - interrupt service routines should be fast, otherwise you get smothing like
//     "Guru Meditation Error: Core 1 panic'ed (Interrupt wdt timeout on CPU1)".

// So we handle the ESP32 interrupt in the following way:
//   - interrupt service routine performs a xSemaphoreGive on mISRSemaphore of can driver
//   - this activates the myESP32Task task that performs "isr_core" that is done
//     by interrupt service routine in "usual" Arduino;
//   - as this task runs in parallel with setup / loop routines, SPI access is natively
//     protected by the beginTransaction / endTransaction pair, that manages a mutex.

//------------------------------------------------------------------------------

[[noreturn]] static void myESP32Task(void* pData) {
    auto* canDriver = static_cast<ACAN2515*>(pData);
    while (true) {
        while (canDriver->paused()) {vTaskDelay(1 / portTICK_PERIOD_MS);}
        canDriver->attachMCP2515InterruptPin();
        xSemaphoreTake(canDriver->mISRSemaphore, portMAX_DELAY);
        bool loop = true;
        while (loop) {
            while (canDriver->paused()) {vTaskDelay(1 / portTICK_PERIOD_MS);}
            loop = canDriver->isr_core();
        }
    }
}

//------------------------------------------------------------------------------

void ACAN2515::attachMCP2515InterruptPin() const {
    if (mINT != 255) {
        gpio_isr_handler_add(mINT, mInterruptServiceRoutine, nullptr);
    }
}

//------------------------------------------------------------------------------
//   CONSTRUCTOR, HARDWARE SPI
//------------------------------------------------------------------------------

ACAN2515::ACAN2515(const spi_device_handle_t& device, // Hardware SPI object
                   const gpio_num_t inINT) : // INT output of MCP2515
    mDevice(device),
    // mSPISettings(10UL * 1000UL * 1000UL, MSBFIRST, SPI_MODE0), // 10 MHz, UL suffix is required for Arduino Uno
    mINT(inINT),
    mRolloverEnable(false),
    mISRSemaphore(xSemaphoreCreateCounting(10, 0)),
    mAccessSemaphore(xSemaphoreCreateMutex()),
    mReceiveBuffer(),
    mCallBackFunctionArray(),
    mTXBIsFree() {
    for (auto& i : mCallBackFunctionArray) {
        i = nullptr;
    }
}

void ACAN2515::setDeviceHandle(const spi_device_handle_t &device) {
    this->mDevice = device;
}

//------------------------------------------------------------------------------
//   BEGIN
//------------------------------------------------------------------------------

uint16_t ACAN2515::begin(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine) {
    return beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine, ACAN2515Mask(), ACAN2515Mask(), NULL, 0);
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::begin(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         const ACAN2515Mask inRXM0,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount == 0 || inAcceptanceFilterCount > 2) {
        errorCode = kOneFilterMaskRequiresOneOrTwoAcceptanceFilters;
    } else if (inAcceptanceFilters == nullptr) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        errorCode = beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine,
                                            inRXM0, inRXM0, inAcceptanceFilters, inAcceptanceFilterCount);
    }
    return errorCode;
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::begin(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         const ACAN2515Mask inRXM0,
                         const ACAN2515Mask inRXM1,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount < 3) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilterCount > 6) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilters == nullptr) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        errorCode = beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine,
                                            inRXM0, inRXM1, inAcceptanceFilters, inAcceptanceFilterCount);
    }
    return errorCode;
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::beginWithoutFilterCheck(const ACAN2515Settings& inSettings,
                                           gpio_isr_t inInterruptServiceRoutine,
                                           const ACAN2515Mask inRXM0,
                                           const ACAN2515Mask inRXM1,
                                           const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                           const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0; // Means no error
    //----------------------------------- Check interrupt service routine is not null
    if ((mINT != 255) && (inInterruptServiceRoutine == nullptr)) {
        errorCode |= kISRIsNull;
    }
    //----------------------------------- Check consistency between ISR and INT pin
    if ((mINT == 255) && (inInterruptServiceRoutine != nullptr)) {
        errorCode |= kISRNotNullAndNoIntPin;
    }
    //----------------------------------- if no error, configure port and MCP2515
    if (errorCode == 0) {
        //--- Configure ports
        if (mINT != 255 && !driverInitialised) {
            // 255 means interrupt is not used
            gpio_config_t io_conf{
                .pin_bit_mask = (1ULL << mINT),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_NEGEDGE
            };
            gpio_config(&io_conf);
            gpio_install_isr_service(0);
        }
        //--- Send software reset to MCP2515
        xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
        write2515Command(RESET_COMMAND);
        xSemaphoreGive(mAccessSemaphore);
        //--- DS20001801J, page 55: The Oscillator Start-up Timer keeps the device in a Reset
        // state for 128 OSC1 clock cycles after the occurrence of a Power-on Reset, SPI Reset,
        // after the assertion of the RESET pin, and after a wake-up from Sleep mode
        // Fot a 1 MHz clock --> 128 µs
        // So delayMicroseconds (10) is too short --> use delay (2)
        //    delayMicroseconds (10) ; // Removed in release 2.1.2
        vTaskDelay(2 / portTICK_PERIOD_MS);
        //--- Internal begin
        errorCode = internalBeginOperation(inSettings,
                                           inRXM0,
                                           inRXM1,
                                           inAcceptanceFilters,
                                           inAcceptanceFilterCount);
    }
    //--- Configure interrupt only if no error (thanks to mvSarma)
    if (errorCode == 0) {
        if (mINT != 255) {
            // 255 means interrupt is not used
            mInterruptServiceRoutine = inInterruptServiceRoutine;
        }
        if (!driverInitialised) {
            xTaskCreate(myESP32Task, "ACAN2515Handler", 2048, this, 16, NULL);
        }
        driverInitialised = true;
    }
    //----------------------------------- Return
    return errorCode;
}

//------------------------------------------------------------------------------
//   MESSAGE RECEPTION
//------------------------------------------------------------------------------

bool ACAN2515::available() {
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const bool hasReceivedMessage = mReceiveBuffer.count() > 0;
    xSemaphoreGive(mAccessSemaphore);
    return hasReceivedMessage;
}

//------------------------------------------------------------------------------

bool ACAN2515::receive(CANMessage& outMessage) {
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const bool hasReceivedMessage = mReceiveBuffer.remove(outMessage);
    xSemaphoreGive(mAccessSemaphore);
    return hasReceivedMessage;
}

//------------------------------------------------------------------------------

bool ACAN2515::dispatchReceivedMessage(const tFilterMatchCallBack inFilterMatchCallBack) {
    CANMessage receivedMessage;
    const bool hasReceived = receive(receivedMessage);
    if (hasReceived) {
        const uint8_t filterIndex = receivedMessage.idx;
        if (nullptr != inFilterMatchCallBack) {
            inFilterMatchCallBack(filterIndex);
        }
        ACANCallBackRoutine callBackFunction = mCallBackFunctionArray[filterIndex];
        if (nullptr != callBackFunction) {
            callBackFunction(receivedMessage);
        }
    }
    return hasReceived;
}

//------------------------------------------------------------------------------
//  INTERRUPTS ARE DISABLED WHEN THESE FUNCTIONS ARE EXECUTED
//------------------------------------------------------------------------------

uint16_t ACAN2515::internalBeginOperation(const ACAN2515Settings& inSettings,
                                          const ACAN2515Mask inRXM0,
                                          const ACAN2515Mask inRXM1,
                                          const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                          const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0; // Ok be default
    //----------------------------------- Check if MCP2515 is accessible
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    write2515Register(CNF1_REGISTER, 0x55);
    bool ok = read2515Register(CNF1_REGISTER) == 0x55;
    if (ok) {
        write2515Register(CNF1_REGISTER, 0xAA);
        ok = read2515Register(CNF1_REGISTER) == 0xAA;
    }
    if (!ok) {
        errorCode = kNoMCP2515;
    }
    xSemaphoreGive(mAccessSemaphore);
    //----------------------------------- Check if settings are correct
    if (!inSettings.mBitRateClosedToDesiredRate) {
        errorCode |= kTooFarFromDesiredBitRate;
    }
    if (inSettings.CANBitSettingConsistency() != 0) {
        errorCode |= kInconsistentBitRateSettings;
    }
    //----------------------------------- Allocate buffer
    if (!mReceiveBuffer.initWithSize(inSettings.mReceiveBufferSize)) {
        errorCode |= kCannotAllocateReceiveBuffer;
    }
    if (!mTransmitBuffer[0].initWithSize(inSettings.mTransmitBuffer0Size)) {
        errorCode |= kCannotAllocateTransmitBuffer0;
    }
    if (!mTransmitBuffer[1].initWithSize(inSettings.mTransmitBuffer1Size)) {
        errorCode |= kCannotAllocateTransmitBuffer1;
    }
    if (!mTransmitBuffer[2].initWithSize(inSettings.mTransmitBuffer2Size)) {
        errorCode |= kCannotAllocateTransmitBuffer2;
    }
    mTXBIsFree[0] = true;
    mTXBIsFree[1] = true;
    mTXBIsFree[2] = true;
    //----------------------------------- If ok, perform configuration
    if (errorCode == 0) {
        //----------------------------------- Set CNF3, CNF2, CNF1 and CANINTE registers
        uint8_t txBuffer[6] = {};
        txBuffer[0] = WRITE_COMMAND;
        txBuffer[1] = CNF3_REGISTER;

        //--- Register CNF3:
        //  Bit 7: SOF
        //  bit 6 --> 0: No Wake-up Filter bit
        //  Bit 5-3: -
        //  Bit 2-0: PHSEG2 - 1
        const uint8_t cnf3 =
                ((inSettings.mCLKOUT_SOF_pin == ACAN2515Settings::SOF) << 6) /* SOF */ |
                ((inSettings.mPhaseSegment2 - 1) << 0) /* PHSEG2 */
            ;
        txBuffer[2] = cnf3;

        //--- Register CNF2:
        //  Bit 7 --> 1: BLTMODE
        //  bit 6: SAM
        //  Bit 5-3: PHSEG1 - 1
        //  Bit 2-0: PRSEG - 1
        const uint8_t cnf2 =
                0x80 /* BLTMODE */ |
                (inSettings.mTripleSampling << 6) /* SAM */ |
                ((inSettings.mPhaseSegment1 - 1) << 3) /* PHSEG1 */ |
                ((inSettings.mPropagationSegment - 1) << 0) /* PRSEG */
            ;
        txBuffer[3] = cnf2;

        //--- Register CNF1:
        //  Bit 7-6: SJW - 1
        //  Bit 5-0: BRP - 1
        const uint8_t cnf1 =
                ((inSettings.mSJW - 1) << 6) /* SJW */ | // Incorrect SJW setting fixed in 2.0.1
                ((inSettings.mBitRatePrescaler - 1) << 0) /* BRP */
            ;
        txBuffer[4] = cnf1;

        //--- Register CANINTE: activate interrupts
        //  Bit 7 --> 0: MERRE
        //  Bit 6 --> 0: WAKIE
        //  Bit 5 --> 0: ERRIE
        //  Bit 4 --> 1: TX2IE
        //  Bit 3 --> 1: TX1IE
        //  Bit 2 --> 1: TX0IE
        //  Bit 1 --> 1: RX1IE
        //  Bit 0 --> 1: RX0IE
        txBuffer[5] = 0x1F;

        spi_transaction_t transaction {
            .length = 48,
            .tx_buffer = txBuffer
        };

        xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
        // ---------------------------------- Data prepared, transmit
        spi_device_polling_transmit(mDevice, &transaction);
        //----------------------------------- Deactivate the RXnBF Pins (High Impedance State)
        write2515Register(BFPCTRL_REGISTER, 0);
        //----------------------------------- Set TXnRTS as inputs
        write2515Register(TXRTSCTRL_REGISTER, 0);
        //----------------------------------- RXBnCTRL
        mRolloverEnable = inSettings.mRolloverEnable;
        const uint8_t acceptAll = (inAcceptanceFilterCount == 0) ? 0x60 : 0x00;
        write2515Register(RXB0CTRL_REGISTER, acceptAll | (uint8_t(inSettings.mRolloverEnable) << 2));
        write2515Register(RXB1CTRL_REGISTER, acceptAll);
        //----------------------------------- Setup mask registers
        setupMaskRegister(inRXM0, RXM0SIDH_REGISTER);
        setupMaskRegister(inRXM1, RXM1SIDH_REGISTER);
        if (inAcceptanceFilterCount > 0) {
            uint8_t idx = 0;
            while (idx < inAcceptanceFilterCount) {
                setupMaskRegister(inAcceptanceFilters[idx].mMask, RXFSIDH_REGISTER[idx]);
                mCallBackFunctionArray[idx] = inAcceptanceFilters[idx].mCallBack;
                idx += 1;
            }
            while (idx < 6) {
                setupMaskRegister(inAcceptanceFilters[inAcceptanceFilterCount - 1].mMask, RXFSIDH_REGISTER[idx]);
                mCallBackFunctionArray[idx] = inAcceptanceFilters[inAcceptanceFilterCount - 1].mCallBack;
                idx += 1;
            }
        }
        //----------------------------------- Set TXBi priorities
        write2515Register(TXB0CTRL_REGISTER, inSettings.mTXBPriority & 3);
        write2515Register(TXB1CTRL_REGISTER, (inSettings.mTXBPriority >> 2) & 3);
        write2515Register(TXB2CTRL_REGISTER, (inSettings.mTXBPriority >> 4) & 3);
        xSemaphoreGive(mAccessSemaphore);
        //----------------------------------- Reset device to requested mode
        uint8_t canctrl = inSettings.mOneShotModeEnabled ? (1 << 3) : 0;
        switch (inSettings.mCLKOUT_SOF_pin) {
        case ACAN2515Settings::CLOCK:
            canctrl |= 0x04 | 0x00; // Same as default setting
            break ;
        case ACAN2515Settings::CLOCK2:
            canctrl |= 0x04 | 0x01;
            break ;
        case ACAN2515Settings::CLOCK4:
            canctrl |= 0x04 | 0x02;
            break ;
        case ACAN2515Settings::CLOCK8:
            canctrl |= 0x04 | 0x03;
            break ;
        case ACAN2515Settings::SOF:
            canctrl |= 0x04;
            break ;
        case ACAN2515Settings::HiZ:
            break ;
        }
        //--- Request mode
        const auto requestedMode = static_cast<uint8_t>(inSettings.mRequestedMode);
        errorCode |= setRequestedMode(canctrl | requestedMode);
    }
    //-----------------------------------
    return errorCode;
}

//------------------------------------------------------------------------------
//   setRequestedMode (private method)
//------------------------------------------------------------------------------

uint16_t ACAN2515::setRequestedMode(const uint8_t inCANControlRegister) {
    uint16_t errorCode = 0;
    //--- Request mode
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    write2515Register(CANCTRL_REGISTER, inCANControlRegister);
    xSemaphoreGive(mAccessSemaphore);
    //--- Wait until requested mode is reached (during 1 or 2 ms)
    bool wait = true;
    const uint32_t deadline = millis() + 2;
    while (wait) {
        xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
        const uint8_t actualMode = read2515Register(CANSTAT_REGISTER) & 0xE0;
        xSemaphoreGive(mAccessSemaphore);
        wait = actualMode != (inCANControlRegister & 0xE0);
        if (wait && (millis() >= deadline)) {
            errorCode |= kRequestedModeTimeOut;
            wait = false;
        }
    }
    //---
    return errorCode;
}

//------------------------------------------------------------------------------
//    Change Mode
//------------------------------------------------------------------------------

uint16_t ACAN2515::changeModeOnTheFly(const ACAN2515Settings::RequestedMode inRequestedMode) {
    //--- Read current mode register (for saving settings of bits 0 ... 4)
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const uint8_t currentMode = read2515Register(CANCTRL_REGISTER);
    xSemaphoreGive(mAccessSemaphore);
    //--- New mode
    const uint8_t newMode = (currentMode & 0x1F) | static_cast<uint8_t>(inRequestedMode);
    //--- Set new mode
    const uint16_t errorCode = setRequestedMode(newMode);
    //---
    return errorCode;
}

//------------------------------------------------------------------------------
//    Set filters on the fly
//------------------------------------------------------------------------------

uint16_t ACAN2515::setFiltersOnTheFly() {
    return internalSetFiltersOnTheFly(ACAN2515Mask(), ACAN2515Mask(), NULL, 0);
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::setFiltersOnTheFly(const ACAN2515Mask inRXM0,
                                      const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                      const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount == 0 || inAcceptanceFilterCount > 2) {
        errorCode = kOneFilterMaskRequiresOneOrTwoAcceptanceFilters;
    } else if (inAcceptanceFilters == nullptr) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        errorCode = internalSetFiltersOnTheFly(inRXM0, ACAN2515Mask(), inAcceptanceFilters, inAcceptanceFilterCount);
    }
    return errorCode;
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::setFiltersOnTheFly(const ACAN2515Mask inRXM0,
                                      const ACAN2515Mask inRXM1,
                                      const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                      const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount < 3) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilterCount > 6) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilters == NULL) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        errorCode = internalSetFiltersOnTheFly(inRXM0, inRXM1, inAcceptanceFilters, inAcceptanceFilterCount);
    }
    return errorCode;
}

//------------------------------------------------------------------------------

uint16_t ACAN2515::internalSetFiltersOnTheFly(const ACAN2515Mask inRXM0,
                                              const ACAN2515Mask inRXM1,
                                              const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                                              const uint8_t inAcceptanceFilterCount) {
    //--- Read current mode register
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const uint8_t currentMode = read2515Register(CANCTRL_REGISTER);
    xSemaphoreGive(mAccessSemaphore);
    //--- Request configuration mode
    const uint8_t configurationMode = (currentMode & 0x1F) | (0b100 << 5); // Preserve bits 0 ... 4
    uint16_t errorCode = setRequestedMode(configurationMode);
    //--- Setup mask registers
    if (errorCode == 0) {
        const uint8_t acceptAll = (inAcceptanceFilterCount == 0) ? 0x60 : 0x00;
        write2515Register(RXB0CTRL_REGISTER, acceptAll | (uint8_t(mRolloverEnable) << 2));
        write2515Register(RXB1CTRL_REGISTER, acceptAll);
        setupMaskRegister(inRXM0, RXM0SIDH_REGISTER);
        setupMaskRegister(inRXM1, RXM1SIDH_REGISTER);
        if (inAcceptanceFilterCount > 0 && inAcceptanceFilters != nullptr) {
            uint8_t idx = 0;
            while (idx < inAcceptanceFilterCount) {
                setupMaskRegister(inAcceptanceFilters[idx].mMask, RXFSIDH_REGISTER[idx]);
                mCallBackFunctionArray[idx] = inAcceptanceFilters[idx].mCallBack;
                idx += 1;
            }
            while (idx < 6) {
                setupMaskRegister(inAcceptanceFilters[inAcceptanceFilterCount - 1].mMask, RXFSIDH_REGISTER[idx]);
                mCallBackFunctionArray[idx] = inAcceptanceFilters[inAcceptanceFilterCount - 1].mCallBack;
                idx += 1;
            }
        }
    }
    //--- Restore saved mode
    if (errorCode == 0) {
        errorCode = setRequestedMode(currentMode);
    }
    //---
    return errorCode;
}

//------------------------------------------------------------------------------
//    end
//------------------------------------------------------------------------------

void ACAN2515::end() {
    //--- Remove interrupt capability of mINT pin
    if (mINT != 255) {
        gpio_isr_handler_remove(mINT);
    }
    //--- Request configuration mode
    constexpr uint8_t configurationMode = (0b100 << 5);
    const uint16_t errorCode __attribute__((unused)) = setRequestedMode(configurationMode);
    //--- Deallocate driver buffers
    mTransmitBuffer[0].free();
    mTransmitBuffer[1].free();
    mTransmitBuffer[2].free();
    mReceiveBuffer.free();
}

uint16_t ACAN2515::resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine) {
    end();
    driverPaused = true;
    const auto ret = beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine, ACAN2515Mask(), ACAN2515Mask(), NULL, 0);
    driverPaused = false;
    return ret;
}

uint16_t ACAN2515::resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         const ACAN2515Mask inRXM0,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount == 0 || inAcceptanceFilterCount > 2) {
        errorCode = kOneFilterMaskRequiresOneOrTwoAcceptanceFilters;
    } else if (inAcceptanceFilters == nullptr) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        driverPaused = true;
        end();
        errorCode = beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine,
                                            inRXM0, inRXM0, inAcceptanceFilters, inAcceptanceFilterCount);
        driverPaused = false;
    }
    return errorCode;
}

uint16_t ACAN2515::resetDriver(const ACAN2515Settings& inSettings,
                         gpio_isr_t inInterruptServiceRoutine,
                         const ACAN2515Mask inRXM0,
                         const ACAN2515Mask inRXM1,
                         const ACAN2515AcceptanceFilter inAcceptanceFilters[],
                         const uint8_t inAcceptanceFilterCount) {
    uint16_t errorCode = 0;
    if (inAcceptanceFilterCount < 3) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilterCount > 6) {
        errorCode = kTwoFilterMasksRequireThreeToSixAcceptanceFilters;
    } else if (inAcceptanceFilters == nullptr) {
        errorCode = kAcceptanceFilterArrayIsNULL;
    } else {
        driverPaused = true;
        end();
        errorCode = beginWithoutFilterCheck(inSettings, inInterruptServiceRoutine,
                                            inRXM0, inRXM1, inAcceptanceFilters, inAcceptanceFilterCount);
        driverPaused = false;
    }
    return errorCode;
}

//------------------------------------------------------------------------------
//    POLLING (ESP32)
//------------------------------------------------------------------------------

void ACAN2515::poll() const {
    xSemaphoreGive(mISRSemaphore);
}

//------------------------------------------------------------------------------
//   INTERRUPT SERVICE ROUTINE (ESP32)
// https://stackoverflow.com/questions/51750377/how-to-disable-interrupt-watchdog-in-esp32-or-increase-isr-time-limit
//------------------------------------------------------------------------------

void IRAM_ATTR ACAN2515::isr() const {
    gpio_isr_handler_remove(mINT);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(mISRSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

//------------------------------------------------------------------------------

bool ACAN2515::isr_core() {
    bool handled = false;
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    uint8_t itStatus = read2515Register(CANSTAT_REGISTER) & 0x0E;
    while (itStatus != 0) {
        handled = true;
        switch (itStatus) {
        case 0: // No interrupt
            break ;
        case 1 << 1: // Error interrupt
            bitModify2515Register(CANINTF_REGISTER, 0x20, 0); // Ack interrupt
            break ;
        case 2 << 1: // Wake-up interrupt
            bitModify2515Register(CANINTF_REGISTER, 0x40, 0); // Ack interrupt
            break ;
        case 3 << 1: // TXB0 interrupt
            handleTXBInterrupt(0);
            break ;
        case 4 << 1: // TXB1 interrupt
            handleTXBInterrupt(1);
            break ;
        case 5 << 1: // TXB2 interrupt
            handleTXBInterrupt(2);
            break ;
        case 6 << 1: // RXB0 interrupt
        case 7 << 1: // RXB1 interrupt
            handleRXBInterrupt();
            break ;
        }
        itStatus = read2515Register(CANSTAT_REGISTER) & 0x0E;
    }
    xSemaphoreGive(mAccessSemaphore);
    return handled;
}

//------------------------------------------------------------------------------
// This function is called by ISR when a MCP2515 receive buffer becomes full

void ACAN2515::handleRXBInterrupt() {
    const uint8_t rxStatus = read2515RxStatus(); // Bit 6: message in RXB0, bit 7: message in RXB1
    const bool received = (rxStatus & 0xC0) != 0;
    if (received) {
        // Message in RXB0 and / or RXB1
        const bool accessRXB0 = (rxStatus & 0x40) != 0;
        CANMessage message;
        //--- Set idx field to matching receive filter
        message.idx = rxStatus & 0x07;
        if (message.idx > 5) {
            message.idx -= 6;
        }
        //---
        uint8_t buffer[6] = {};
        uint8_t dataBuffer[8] = {};
        spi_transaction_t getDataInitial{
            .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_CS_KEEP_ACTIVE,
            .length = 8,
            .rxlength = 40,
            .tx_data = {accessRXB0 ? READ_FROM_RXB0SIDH_COMMAND : READ_FROM_RXB1SIDH_COMMAND},
            .rx_buffer = buffer
        };

        spi_transaction_t getDataFinal{
            .rx_buffer = dataBuffer
        };

        spi_device_acquire_bus(mDevice, portMAX_DELAY);
        // Transmit data to read first 5 bytes
        const esp_err_t err = spi_device_polling_transmit(mDevice, &getDataInitial);
        if (err == ESP_OK) {
            // Extract data length and read the remaining data
            message.len = buffer[4] & 0x0F;
            getDataFinal.rxlength = message.len * 8;
            spi_device_polling_transmit(mDevice, &getDataFinal);
            spi_device_release_bus(mDevice);

            // Interpret the data
            message.id = buffer[0] << 3; //--- SIDH
            message.id |= buffer[1] >> 5; //--- SIDL
            message.ext = (buffer[1] & 0x08) != 0;
            if (message.ext) {
                message.id <<= 2;
                message.id |= (buffer[1] & 0x03);
                message.id <<= 8;
                message.id |= buffer[2];
                message.id <<= 8;
                message.id |= buffer[3];
                message.rtr = (buffer[4] & 0x40) != 0; // RTR bit in DLC is significant only for extended frame
            } else {
                message.rtr = (buffer[1] & 0x10) != 0; // Only significant for standard frame
            }

            for (int i = 0; i < message.len; i++) {
                message.data[i] = dataBuffer[i];
            }
        } else {
            // Release the bus and stop processing on failure
            spi_device_release_bus(mDevice);
            return;
        }
        //--- Free receive buffer command
        bitModify2515Register(CANINTF_REGISTER, accessRXB0 ? 0x01 : 0x02, 0);
        //--- Enter received message in receive buffer (if not full)
        mReceiveBuffer.append(message);
    }
}

//------------------------------------------------------------------------------
// This function is called by ISR when a MCP2515 transmit buffer becomes empty

void ACAN2515::handleTXBInterrupt(const uint8_t inTXB) {
    // inTXB value is 0, 1 or 2
    //--- Acknowledge interrupt
    bitModify2515Register(CANINTF_REGISTER, 0x04 << inTXB, 0);
    //--- Send an other message ?
    CANMessage message;
    const bool ok = mTransmitBuffer[inTXB].remove(message);
    if (ok) {
        internalSendMessage(message, inTXB);
    } else {
        mTXBIsFree[inTXB] = true;
    }
}

//------------------------------------------------------------------------------

void ACAN2515::internalSendMessage(const CANMessage& inFrame, const uint8_t inTXB) const {
    // inTXB is 0, 1 or 2
    //--- Send command
    //      send via TXB0: 0x81
    //      send via TXB1: 0x82
    //      send via TXB2: 0x84
    const uint8_t sendCommand = REQUEST_TO_SEND_COMMAND | (1 << inTXB);
    //--- Load TX buffer command
    //      Load TXB0, start at TXB0SIDH: 0x40
    //      Load TXB1, start at TXB1SIDH: 0x42
    //      Load TXB2, start at TXB2SIDH: 0x44
    const uint8_t loadTxBufferCommand = LOAD_TX_BUFFER_COMMAND | (inTXB << 1);
    //--- Send message
    uint8_t transmitBuffer[14] = {};
    transmitBuffer[0] = loadTxBufferCommand;
    if (inFrame.ext) {
        uint32_t v = inFrame.id >> 21;
        transmitBuffer[1] = static_cast<uint8_t>(v); // ID28 ... ID21 --> SIDH
        v = (inFrame.id >> 13) & 0xE0; // ID20, ID19, ID18 in bits 7, 6, 5
        v |= (inFrame.id >> 16) & 0x03; // ID17, ID16 in bits 1, 0
        v |= 0x08; // Extended bit
        transmitBuffer[2] = static_cast<uint8_t>(v); // ID20, ID19, ID18, -, 1, -, ID17, ID16 --> SIDL
        v = (inFrame.id >> 8) & 0xFF; // ID15, ..., ID8
        transmitBuffer[3] = static_cast<uint8_t>(v); // ID15, ID14, ID13, ID12, ID11, ID10, ID9, ID8 --> EID8
        v = inFrame.id & 0xFF; // ID7, ..., ID0
        transmitBuffer[4] = static_cast<uint8_t>(v); // ID7, ID6, ID5, ID4, ID3, ID2, ID1, ID0 --> EID0
    } else {
        // Standard frame
        uint32_t v = inFrame.id >> 3;
        transmitBuffer[1] = static_cast<uint8_t>(v); // ID10 ... ID3 --> SIDH
        v = (inFrame.id << 5) & 0xE0; // ID2, ID1, ID0 in bits 7, 6, 5
        transmitBuffer[2] = static_cast<uint8_t>(v); // ID2, ID1, ID0, -, 0, -, 0, 0 --> SIDL
        transmitBuffer[3] = 0x00; // any value --> EID8
        transmitBuffer[4] = 0x00; // any value --> EID0
    }
    //--- DLC
    uint8_t v = inFrame.len;
    if (v > 8) {
        v = 8;
    }
    if (inFrame.rtr) {
        v |= 0x40;
    }
    transmitBuffer[5] = v;
    const uint8_t bytesToTransfer = (6 + v) * 8;
    //--- Send data
    if (!inFrame.rtr) {
        for (uint8_t i = 0; i < inFrame.len; i++) {
            transmitBuffer[6 + i] = inFrame.data[i];
        }
    }

    spi_transaction_t transmitData{
        .length = bytesToTransfer,
        .tx_buffer = transmitBuffer,
    };

    spi_device_polling_transmit(mDevice, &transmitData);

    //--- Write send command
    write2515Command(sendCommand);
}

//------------------------------------------------------------------------------
//  INTERNAL SPI FUNCTIONS
//------------------------------------------------------------------------------

void ACAN2515::write2515Command(const uint8_t data) const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_data = {data},
    };

    spi_device_polling_transmit(mDevice, &transmitData);
}

//------------------------------------------------------------------------------

void ACAN2515::write2515Register(const uint8_t inRegister, const uint8_t inValue) const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 24,
        .tx_data = {WRITE_COMMAND, inRegister, inValue},
    };

    spi_device_polling_transmit(mDevice, &transmitData);
}

//------------------------------------------------------------------------------

uint8_t ACAN2515::read2515Register(const uint8_t inRegister) const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .rxlength = 8,
        .tx_data = {READ_COMMAND, inRegister},
    };

    const esp_err_t err = spi_device_polling_transmit(mDevice, &transmitData);
    if (err == ESP_OK) {
        return transmitData.rx_data[0];
    }
    return 0xFF;
}

//------------------------------------------------------------------------------

uint8_t ACAN2515::read2515Status() const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 8,
        .rxlength = 8,
        .tx_data = {READ_STATUS_COMMAND},
    };

    const esp_err_t err = spi_device_polling_transmit(mDevice, &transmitData);
    if (err == ESP_OK) {
        return transmitData.rx_data[0];
    }
    return 0xFF;
}

//------------------------------------------------------------------------------

uint8_t ACAN2515::read2515RxStatus() const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .length = 8,
        .rxlength = 8,
        .tx_data = {RX_STATUS_COMMAND},
    };

    const esp_err_t err = spi_device_polling_transmit(mDevice, &transmitData);
    if (err == ESP_OK) {
        return transmitData.rx_data[0];
    }
    return 0xFF;
}

//------------------------------------------------------------------------------

void ACAN2515::bitModify2515Register(const uint8_t inRegister,
                                     const uint8_t inMask,
                                     const uint8_t inData) const {
    spi_transaction_t transmitData{
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 32,
        .tx_data = {BIT_MODIFY_COMMAND, inRegister, inMask, inData},
    };

    spi_device_polling_transmit(mDevice, &transmitData);
}

//------------------------------------------------------------------------------

void ACAN2515::setupMaskRegister(const ACAN2515Mask inMask, const uint8_t inRegister) const {
    uint8_t transmitBuffer[] = {WRITE_COMMAND, inRegister, inMask.mSIDH, inMask.mSIDL, inMask.mEID8, inMask.mEID0};
    spi_transaction_t transmitData{
        .length = 48,
        .tx_buffer = transmitBuffer,
    };

    spi_device_polling_transmit(mDevice, &transmitData);
}

//------------------------------------------------------------------------------
//   MCP2515 controller state
//------------------------------------------------------------------------------

uint8_t ACAN2515::transmitErrorCounter() const {
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const uint8_t result = read2515Register(TEC_REGISTER);
    xSemaphoreGive(mAccessSemaphore);
    return result;
}

//------------------------------------------------------------------------------

uint8_t ACAN2515::receiveErrorCounter() const {
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const uint8_t result = read2515Register(REC_REGISTER);
    xSemaphoreGive(mAccessSemaphore);
    return result;
}

//------------------------------------------------------------------------------

uint8_t ACAN2515::errorFlagRegister() const {
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const uint8_t result = read2515Register(EFLG_REGISTER);
    xSemaphoreGive(mAccessSemaphore);
    return result;
}

//------------------------------------------------------------------------------
//   MESSAGE EMISSION
//------------------------------------------------------------------------------

bool ACAN2515::sendBufferNotFullForIndex(const uint32_t inIndex) const {
    //--- Fix send buffer index
    uint8_t idx = inIndex;
    if (idx > 2) {
        idx = 0;
    }
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    const bool ok = mTXBIsFree[idx] || !mTransmitBuffer[idx].isFull();
    xSemaphoreGive(mAccessSemaphore);
    return ok;
}

//------------------------------------------------------------------------------

bool ACAN2515::tryToSend(const CANMessage& inMessage) {
    //--- Fix send buffer index
    uint8_t idx = inMessage.idx;
    if (idx > 2) {
        idx = 0;
    }
    //--- Bug fix in 2.0.6 (thanks to Fergus Duncan): interrupts were only disabled for Teensy boards
    //---
    xSemaphoreTake(mAccessSemaphore, portMAX_DELAY);
    bool ok = mTXBIsFree[idx];
    if (ok) {
        // Transmit buffer and TXB are both free: transmit immediatly
        mTXBIsFree[idx] = false;
        internalSendMessage(inMessage, idx);
    } else {
        // Enter in transmit buffer, if not full
        ok = mTransmitBuffer[idx].append(inMessage);
    }
    xSemaphoreGive(mAccessSemaphore);
    return ok;
}

//------------------------------------------------------------------------------
