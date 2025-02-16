//----------------------------------------------------------------------------------------
// MCP2515 Receive filter classes
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//----------------------------------------------------------------------------------------

#pragma once

//----------------------------------------------------------------------------------------

#include <cstdint>
//----------------------------------------------------------------------------------------

class ACAN2515Mask {
    //--- Default constructor
public:
    ACAN2515Mask(void) :
        mSIDH(0),
        mSIDL(0),
        mEID8(0),
        mEID0(0) {}

    //--- Properties
    uint8_t mSIDH;
    uint8_t mSIDL;
    uint8_t mEID8;
    uint8_t mEID0;
};

//----------------------------------------------------------------------------------------

class ACAN2515AcceptanceFilter {
public:
    typedef void (*tCallBackRoutine)(const CANMessage& inMessage);

    const ACAN2515Mask mMask;

    const tCallBackRoutine mCallBack;
};

//----------------------------------------------------------------------------------------

inline ACAN2515Mask standard2515Mask(const uint16_t inIdentifier,
                                     const uint8_t inByte0,
                                     const uint8_t inByte1) {
    ACAN2515Mask result;
    result.mSIDH = static_cast<uint8_t>(inIdentifier >> 3);
    result.mSIDL = static_cast<uint8_t>(inIdentifier << 5);
    result.mEID8 = inByte0;
    result.mEID0 = inByte1;
    return result;
}

//----------------------------------------------------------------------------------------

inline ACAN2515Mask extended2515Mask(const uint32_t inIdentifier) {
    ACAN2515Mask result;
    result.mSIDH = static_cast<uint8_t>(inIdentifier >> 21);
    result.mSIDL = static_cast<uint8_t>(((inIdentifier >> 16) & 0x03) | ((inIdentifier >> 13) & 0xE0));
    result.mEID8 = static_cast<uint8_t>(inIdentifier >> 8);
    result.mEID0 = static_cast<uint8_t>(inIdentifier);
    return result;
}

//----------------------------------------------------------------------------------------

inline ACAN2515Mask standard2515Filter(const uint16_t inIdentifier,
                                       const uint8_t inByte0,
                                       const uint8_t inByte1) {
    ACAN2515Mask result;
    result.mSIDH = static_cast<uint8_t>(inIdentifier >> 3);
    result.mSIDL = static_cast<uint8_t>(inIdentifier << 5);
    result.mEID8 = inByte0;
    result.mEID0 = inByte1;
    return result;
}

//----------------------------------------------------------------------------------------

inline ACAN2515Mask extended2515Filter(const uint32_t inIdentifier) {
    ACAN2515Mask result;
    result.mSIDH = static_cast<uint8_t>(inIdentifier >> 21);
    result.mSIDL = static_cast<uint8_t>(((inIdentifier >> 16) & 0x03) | ((inIdentifier >> 13) & 0xE0)) | 0x08;
    result.mEID8 = static_cast<uint8_t>(inIdentifier >> 8);
    result.mEID0 = static_cast<uint8_t>(inIdentifier);
    return result;
}

//----------------------------------------------------------------------------------------
