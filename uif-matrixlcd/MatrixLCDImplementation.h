/* mbed
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __UIF_MATRIXLCD_IMPLEMENTATION_H__
#define __UIF_MATRIXLCD_IMPLEMENTATION_H__

#include "mbed-drivers/mbed.h"

#include "uif-matrixlcd/MatrixLCDBase.h"
#include "uif-framebuffer-1bit/FrameBuffer1Bit.h"

using namespace mbed::util;
using namespace uif;

class MatrixLCDImplementation : MatrixLCDBase
{
public:
    MatrixLCDImplementation(void);
    ~MatrixLCDImplementation(void);

    // pointer to the first pixels
    virtual SharedPointer<FrameBuffer> getFrameBuffer();

    // send the front buffer to the screen
    virtual void sendFrameBuffer(SharedPointer<FrameBuffer>& buffer, FunctionPointer onTransferBegin, FunctionPointer onTransferDone);

private:
    void transfer(uint16_t* buffer);
    void transferDone(Buffer, Buffer, int);
    void init(void);
    void toggleCom(void);

    // disable copy and assignment constructor
    MatrixLCDImplementation(const MatrixLCDImplementation&);
    MatrixLCDImplementation& operator=(const MatrixLCDImplementation&);

private:
    SPI& spi;
    DigitalOut cs;
    DigitalOut disp;
    DigitalOut* extComIn;

    FunctionPointer onTransferBegin;
    FunctionPointer onTransferDone;

    SharedPointer<FrameBuffer> frameBuffer[2];
    uint8_t frameIndex;
};


#endif // __UIF_MATRIXLCD_IMPLEMENTATION_H__
