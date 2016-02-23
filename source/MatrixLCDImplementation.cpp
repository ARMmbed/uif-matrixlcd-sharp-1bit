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

#include "mbed-drivers/mbed.h"

#include "uif-matrixlcd/MatrixLCDImplementation.h"
#include "wrd-output-compare/OutputCompare.h"
#include "wrd-utilities/SharedModules.h"

#if YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SCREEN_PRESENT
#define LCD_NAME        YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SCREEN_SPI_NAME
#define LCD_CS          YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SCREEN_SPI_CS
#define LCD_DISP        YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SCREEN_PIN_ENABLE
#define LCD_EXT_COM_IN  YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_SCREEN_PIN_REFRESH
#else
#error missing configuration
#endif

/// Private Constants
enum PrivateConstants{
    LCD_Stride_Bytes = 18,  // bytes
    LCD_Width_Bits   = 128, // bits (px) wide
    LCD_Width_Bytes  = 16,  // bytes wide
    LCD_Height_Rows  = 128, // rows (px) tall
    LCD_End_Padding  = 2    // bytes to make sure last line is written
};

/// Private Data
static __attribute__((aligned (4))) uint16_t LCD_Buffer_1[LCD_Stride_Bytes * LCD_Height_Rows / 2 + LCD_End_Padding] = {0};
static __attribute__((aligned (4))) uint16_t LCD_Buffer_2[LCD_Stride_Bytes * LCD_Height_Rows / 2 + LCD_End_Padding] = {0};

static uint16_t* LCD_Front_Buffer = LCD_Buffer_1; // front buffer is displayed
static uint16_t* LCD_Back_Buffer  = LCD_Buffer_2; // back buffer is drawn into

/// Private Function Declarations
static void initControlSignals(uint16_t* buf);
static uint8_t* startOfPixels(uint16_t* buf);

/// Public API Implementation
MatrixLCDImplementation::MatrixLCDImplementation()
    :   MatrixLCDBase(),
        spi(LCD_NAME),
        cs(LCD_CS),
        disp(LCD_DISP),
        extComIn(NULL),
        onTransferBegin(),
        onTransferDone(),
        frameIndex(0)
{
    // switch disp pin on
    disp = 1;

    // configure SPI and OutputCompare in task to decouple from constructor
    FunctionPointer0<void> callback(this, &MatrixLCDImplementation::init);
    minar::Scheduler::postCallback(callback)
        .tolerance(1);

    // write control signals to frame buffer
    initControlSignals(LCD_Front_Buffer);
    initControlSignals(LCD_Back_Buffer);

    // create FrameBuffer1Bit objects from memory buffers
    frameBuffer[0] = SharedPointer<FrameBuffer>(new FrameBuffer1Bit(startOfPixels(LCD_Front_Buffer),
                                                                    LCD_Width_Bits,
                                                                    LCD_Height_Rows,
                                                                    LCD_Stride_Bytes));

    frameBuffer[1] = SharedPointer<FrameBuffer>(new FrameBuffer1Bit(startOfPixels(LCD_Back_Buffer),
                                                                    LCD_Width_Bits,
                                                                    LCD_Height_Rows,
                                                                    LCD_Stride_Bytes));

    // clear buffers
    frameBuffer[0]->drawRectangle(0, LCD_Width_Bits, 0, LCD_Height_Rows, 0);
    frameBuffer[1]->drawRectangle(0, LCD_Width_Bits, 0, LCD_Height_Rows, 0);
}

MatrixLCDImplementation::~MatrixLCDImplementation()
{
    delete extComIn;
}

void MatrixLCDImplementation::init()
{
    // configure SPI
    spi.format(16, 0, SPI_LSB);
    spi.frequency(1000000);
    spi.set_dma_usage(DMA_USAGE_OPPORTUNISTIC);

    // toggle extComIn every second to avoid display burnin.
    bool retval = false;

#if YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_OUTPUT_COMPARE_PRESENT
    retval = wrd::OutputCompare(LCD_EXT_COM_IN, 500);
#endif

    // retval is false if OutputCompare failed.
    if (retval == false)
    {
        // Revert to manual pin toggling if pin is not supported by OutputCompare
        extComIn = new DigitalOut(LCD_EXT_COM_IN);

        minar::Scheduler::postCallback(this, &MatrixLCDImplementation::toggleCom)
            .period(minar::milliseconds(900))
            .tolerance(minar::milliseconds(100));
    }
}

void MatrixLCDImplementation::toggleCom()
{
    *extComIn = !*extComIn;
}

// pointer to the first pixels
// pointer to the first pixels
SharedPointer<FrameBuffer> MatrixLCDImplementation::getFrameBuffer()
{
    frameIndex = frameIndex ^ 0x01;

    return frameBuffer[frameIndex];
}

// send the front buffer to the screen
void MatrixLCDImplementation::sendFrameBuffer(SharedPointer<FrameBuffer>& buffer, FunctionPointer _onTransferBegin, FunctionPointer _onTransferDone)
{
    // store callbacks
    onTransferBegin = _onTransferBegin;
    onTransferDone = _onTransferDone;

    // select buffer
    uint16_t* address = NULL;

    if (buffer.get() == frameBuffer[0].get())
    {
        address = LCD_Front_Buffer;
    }
    else
    {
        address = LCD_Back_Buffer;
    }

    // call transfer function
    FunctionPointer1<void, uint16_t*> fp(this, &MatrixLCDImplementation::transfer);
    minar::Scheduler::postCallback(fp.bind(address))
        .tolerance(minar::milliseconds(0));
}

void MatrixLCDImplementation::transfer(uint16_t* address)
{
    // chip select - note the LCD is active high
    cs = 1;

    // use local transfer done function to deselect CS pin before calling external callback
    SPI::event_callback_t onFinish(this, &MatrixLCDImplementation::transferDone);

    // send buffer over SPI
    spi.transfer()
        .tx(address, (LCD_Height_Rows*LCD_Stride_Bytes + LCD_End_Padding) / 2)
        .callback(onFinish, SPI_EVENT_ALL)
        .apply();

    // call external callback signaling transfer has begun
    if (onTransferBegin)
    {
        onTransferBegin.call();
    }
}

void MatrixLCDImplementation::transferDone(Buffer txBuffer, Buffer rxBuffer, int event)
{
    (void) txBuffer;
    (void) rxBuffer;
    (void) event;

    // de-select screen
    cs = 0;

    // schedule original callback function to be called
    minar::Scheduler::postCallback(onTransferDone)
        .tolerance(minar::milliseconds(0));
}


/// Private Function Definitions

static void initControlSignals(uint16_t* buf)
{
    uint8_t* bufb = (uint8_t*)buf;

    for(int i=0; i<LCD_Height_Rows; i++)
    {
        // use the "update" command to start every line – if this isn't the
        // first line sent within the transfer it will be ignored anyway
        bufb[0] = 0x01;
        // 1-indexed address of line
        bufb[1] = i+1;

        for(int j = 2; j < LCD_Stride_Bytes; j++)
            bufb[j] = 0xa5;

        bufb += LCD_Stride_Bytes;
    }
}

static uint8_t* startOfPixels(uint16_t* buf)
{
    return ((uint8_t*) buf) + 2;
}
