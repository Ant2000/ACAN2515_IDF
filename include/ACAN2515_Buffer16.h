//----------------------------------------------------------------------------------------

#pragma once

//----------------------------------------------------------------------------------------

#include <ACAN2515_CANMessage.h>

//----------------------------------------------------------------------------------------

class ACAN2515_Buffer16 {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Default constructor
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    ACAN2515_Buffer16() :
        mBuffer(nullptr),
        mSize(0),
        mReadIndex(0),
        mCount(0),
        mPeakCount(0) {}

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Destructor
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ~ ACAN2515_Buffer16() {
        delete [] mBuffer;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Private properties
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    CANMessage* mBuffer;
    uint16_t mSize;
    uint16_t mReadIndex;
    uint16_t mCount;
    uint16_t mPeakCount; // > mSize if overflow did occur

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Accessors
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

public:
    bool isFull() const { return mCount == mSize; }
    uint16_t size() const { return mSize; }
    uint16_t count() const { return mCount; }
    uint16_t peakCount() const { return mPeakCount; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // initWithSize
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool initWithSize(const uint16_t inSize) {
        delete [] mBuffer;
        mBuffer = new CANMessage [inSize];
        const bool ok = mBuffer != nullptr;
        mSize = ok ? inSize : 0;
        mReadIndex = 0;
        mCount = 0;
        mPeakCount = 0;
        return ok;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // append
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool append(const CANMessage& inMessage) {
        const bool ok = mCount < mSize;
        if (ok) {
            uint16_t writeIndex = mReadIndex + mCount;
            if (writeIndex >= mSize) {
                writeIndex -= mSize;
            }
            mBuffer[writeIndex] = inMessage;
            mCount++;
            if (mPeakCount < mCount) {
                mPeakCount = mCount;
            }
        }
        return ok;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Remove
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool remove(CANMessage& outMessage) {
        const bool ok = mCount > 0;
        if (ok) {
            outMessage = mBuffer[mReadIndex];
            mCount -= 1;
            mReadIndex += 1;
            if (mReadIndex == mSize) {
                mReadIndex = 0;
            }
        }
        return ok;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Free
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    void free() {
        delete [] mBuffer;
        mBuffer = nullptr;
        mSize = 0;
        mReadIndex = 0;
        mCount = 0;
        mPeakCount = 0;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Reset Peak Count
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    void resetPeakCount() { mPeakCount = mCount; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // No copy
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

private:
    ACAN2515_Buffer16(const ACAN2515_Buffer16&) = delete ;
    ACAN2515_Buffer16& operator =(const ACAN2515_Buffer16&) = delete ;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
};

//----------------------------------------------------------------------------------------
