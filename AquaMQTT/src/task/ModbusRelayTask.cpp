#include "task/ModbusRelayTask.h"

#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/uart_periph.h>

#include "config/Configuration.h"
#include "state/Optitronic2State.h"
#include "task/Optitronic2MQTTTask.h"

using namespace aquamqtt::message::optitronic2;

// External reference to MQTT task for write dequeue
extern aquamqtt::Optitronic2MQTTTask optitronic2MqttTask;

namespace aquamqtt
{

ModbusRelayTask::ModbusRelayTask()
    : mHmiFrameBuffer{}
    , mHmiFrameLength(0)
    , mHmiLastByteTime(0)
    , mHmiFrameInProgress(false)
    , mMainFrameBuffer{}
    , mMainFrameLength(0)
    , mMainLastByteTime(0)
    , mMainFrameInProgress(false)
    , mPendingReadRequest(false)
    , mPendingReadStartReg(0)
    , mPendingReadQuantity(0)
    , mFramesReceived(0)
    , mCrcErrors(0)
    , mFramesRelayed(0)
    , mWritesInjected(0)
    , mHmiFramesIn(0)
    , mMainFramesIn(0)
    , mHmiBytesIn(0)
    , mMainBytesIn(0)
    , mLastStatsUpdate(0)
    , mEchoBytes(0)
    , mTxBytesWritten(0)
    , mLastFwdFrame{}
    , mLastFwdFrameLen(0)
{
}

void ModbusRelayTask::spawn()
{
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(ModbusRelayTask::innerTask, "modbusRelayLoop", 8192, this, 5, &handle, 1);
    // Don't register with WDT for now — testing if WDT kills the task
}

[[noreturn]] void ModbusRelayTask::innerTask(void* pvParameters)
{
    auto* self = static_cast<ModbusRelayTask*>(pvParameters);

    self->setup();

    while (true)
    {
        self->loop();
    }
}

void ModbusRelayTask::setup()
{
    // Serials already opened from main setup()
    // TX enable pins: HIGH=transmit, LOW=receive
    pinMode(config::GPIO_ENABLE_TX_HMI, OUTPUT);
    pinMode(config::GPIO_ENABLE_TX_MAIN, OUTPUT);
    digitalWrite(config::GPIO_ENABLE_TX_HMI, LOW);    // receive from HMI
    digitalWrite(config::GPIO_ENABLE_TX_MAIN, LOW);    // receive from Main

    // === ONE-WIRE UART: Fully disconnect TX pins during receive ===
    // The SN74LVC2T45 in receive mode (DIR=LOW) makes B-side inputs, A-side outputs.
    // The translator drives A1 (our TX pin) with bus data. If ESP's UART TX also drives
    // this pin, there's bus contention. Fix: disconnect UART TX output from pin entirely.
    // Route to SIG_GPIO_OUT_IDX (constant HIGH, pin not driven by peripheral)
    // Actually: just set pin as input and disconnect output signal
    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_TX, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)config::GPIO_HMI_TX, GPIO_MODE_INPUT);
    // Disconnect any output signal from these pins
    esp_rom_gpio_connect_out_signal(config::GPIO_MAIN_TX, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(config::GPIO_HMI_TX, SIG_GPIO_OUT_IDX, false, false);

    // Also disconnect RX pins from output (they should only be inputs)
    // The UART RX input is still connected via gpio_connect_in_signal from Serial.begin()
    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_RX, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)config::GPIO_HMI_RX, GPIO_MODE_INPUT);

    Serial.println("[relay] setup complete");
}

void ModbusRelayTask::loop()
{
    unsigned long now = millis();

    // ===== HMI side: receive frames from HMI controller =====
    if (mHmiFrameInProgress && (now - mHmiLastByteTime) >= MODBUS_FRAME_SILENCE_MS)
    {
        if (mHmiFrameLength >= 5)
        {
            processAndForward(mHmiFrameBuffer, mHmiFrameLength, Serial2, true);
        }
        mHmiFrameLength     = 0;
        mHmiFrameInProgress = false;
    }

    while (Serial1.available())
    {
        uint8_t byte = Serial1.read();
        mHmiBytesIn++;
        now = millis();

        if (mHmiFrameInProgress && (now - mHmiLastByteTime) >= MODBUS_FRAME_SILENCE_MS)
        {
            if (mHmiFrameLength >= 5)
            {
                processAndForward(mHmiFrameBuffer, mHmiFrameLength, Serial2, true);
            }
            mHmiFrameLength     = 0;
            mHmiFrameInProgress = false;
        }

        if (mHmiFrameLength < MODBUS_MAX_FRAME_SIZE)
        {
            mHmiFrameBuffer[mHmiFrameLength++] = byte;
        }
        mHmiLastByteTime    = now;
        mHmiFrameInProgress = true;
    }

    // ===== Main controller side: receive frames from Main controller =====
    if (mMainFrameInProgress && (now - mMainLastByteTime) >= MODBUS_FRAME_SILENCE_MS)
    {
        if (mMainFrameLength >= 5)
        {
            processAndForward(mMainFrameBuffer, mMainFrameLength, Serial1, false);
        }
        mMainFrameLength     = 0;
        mMainFrameInProgress = false;
    }

    while (Serial2.available())
    {
        uint8_t byte = Serial2.read();
        mMainBytesIn++;
        now = millis();

        if (mMainFrameInProgress && (now - mMainLastByteTime) >= MODBUS_FRAME_SILENCE_MS)
        {
            if (mMainFrameLength >= 5)
            {
                processAndForward(mMainFrameBuffer, mMainFrameLength, Serial1, false);
            }
            mMainFrameLength     = 0;
            mMainFrameInProgress = false;
        }

        if (mMainFrameLength < MODBUS_MAX_FRAME_SIZE)
        {
            mMainFrameBuffer[mMainFrameLength++] = byte;
        }
        mMainLastByteTime    = now;
        mMainFrameInProgress = true;
    }

    // ===== Inject pending MQTT write commands during bus idle =====
    if (!mHmiFrameInProgress && !mMainFrameInProgress)
    {
        injectPendingWrites();
    }

    // ===== Periodic stats update =====
    if ((now - mLastStatsUpdate) >= 5000)
    {
        Optitronic2State::getInstance().updateStats(mFramesReceived, mCrcErrors);
        mLastStatsUpdate = now;
    }
}

void ModbusRelayTask::processAndForward(uint8_t* buffer, uint8_t length, HardwareSerial& destination, bool fromHmi)
{
    // Count per-side incoming frames
    if (fromHmi) mHmiFramesIn++;
    else mMainFramesIn++;

    // Save last forwarded frame for debugging
    mLastFwdFrameLen = min((uint8_t)32, length);
    memcpy(mLastFwdFrame, buffer, mLastFwdFrameLen);

    // Parse the frame for state extraction
    parseFrame(buffer, length, fromHmi);

    // Determine pins and UART number for the destination side
    uint8_t txEnablePin;
    uint8_t rxPin;
    int     uartNum;

    if (fromHmi)
    {
        // Forwarding toward Main (Serial2 = UART2)
        txEnablePin = config::GPIO_ENABLE_TX_MAIN;
        rxPin       = config::GPIO_MAIN_RX;  // GPIO 5, connected to SN74LVC2T45 A2
        uartNum     = 2;
    }
    else
    {
        // Forwarding toward HMI (Serial1 = UART1)
        txEnablePin = config::GPIO_ENABLE_TX_HMI;
        rxPin       = config::GPIO_HMI_RX;   // GPIO 7, connected to SN74LVC2T45 A2
        uartNum     = 1;
    }

    // === ONE-WIRE UART FIX ===
    // The SN74LVC2T45 has BOTH B1 and B2 connected to the same one-wire bus.
    // When DIR=HIGH (transmit mode, A→B): A1 drives B1, A2 drives B2.
    // When DIR=LOW (receive mode, B→A): B drives through to A1 and A2.
    //
    // Problem 1: During TX, if A2 (RX pin) isn't driven with TX signal, B1≠B2 = contention
    // Problem 2: During RX, if UART TX output still drives A1, it fights the translator
    //
    // Solution: 
    //   TX mode: connect UART TX signal to BOTH A1 (txPin) and A2 (rxPin)
    //   RX mode: disconnect all output signals, pins are pure inputs
    int txSignal = uart_periph_signal[uartNum].pins[SOC_UART_TX_PIN_IDX].signal;
    int rxSignal = uart_periph_signal[uartNum].pins[SOC_UART_RX_PIN_IDX].signal;
    uint8_t txPin = fromHmi ? config::GPIO_MAIN_TX : config::GPIO_HMI_TX;

    // Connect UART TX output to TX pin (A1)
    gpio_set_direction((gpio_num_t)txPin, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(txPin, txSignal, false, false);

    // Connect UART TX output to RX pin (A2) — both pins carry same data
    gpio_set_direction((gpio_num_t)rxPin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(rxPin, txSignal, false, false);

    // Small delay for GPIO matrix and translator to settle
    delayMicroseconds(10);

    // Enable transmit direction on level translator (DIR=HIGH → A→B)
    digitalWrite(txEnablePin, HIGH);

    // Send frame
    size_t written = destination.write(buffer, length);
    destination.flush();
    mTxBytesWritten += written;

    // Back to receive mode (DIR=LOW → B→A)
    digitalWrite(txEnablePin, LOW);

    // Fully disconnect TX pin from UART output (prevent fighting translator)
    esp_rom_gpio_connect_out_signal(txPin, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)txPin, GPIO_MODE_INPUT);

    // Restore RX pin as UART RX input
    esp_rom_gpio_connect_out_signal(rxPin, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)rxPin, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(rxPin, rxSignal, false);

    // Clear any echo bytes that appeared in RX buffer during transmit
    delayMicroseconds(200);
    while (destination.available()) { destination.read(); mEchoBytes++; }

    mFramesRelayed++;
}

void ModbusRelayTask::parseFrame(uint8_t* buffer, uint8_t length, bool fromHmi)
{
    ModbusFrame frame;

    if (!parseModbusFrame(buffer, length, frame))
    {
        mCrcErrors++;
        return;
    }

    mFramesReceived++;

    if (frame.slaveAddress != MODBUS_SLAVE_ADDRESS)
    {
        return;
    }

    switch (frame.functionCode)
    {
        case FC_READ_HOLDING_REGISTERS:
        {
            if (frame.dataLength == 4 && fromHmi)
            {
                // HMI read request
                ModbusReadRequest request;
                if (parseReadRequest(frame, request))
                {
                    mPendingReadRequest  = true;
                    mPendingReadStartReg = request.startRegister;
                    mPendingReadQuantity = request.quantity;
                }
            }
            else if (frame.dataLength >= 3 && !fromHmi && mPendingReadRequest)
            {
                // Main controller read response
                ModbusReadResponse response;
                if (parseReadResponse(frame, response))
                {
                    if (response.registerCount == mPendingReadQuantity)
                    {
                        Optitronic2State::getInstance().storeRegisters(
                                mPendingReadStartReg, response.registers, response.registerCount);
                    }
                }
                mPendingReadRequest = false;
            }
            break;
        }

        case FC_WRITE_SINGLE_REGISTER:
        {
            ModbusWriteSingle write;
            if (parseWriteSingle(frame, write))
            {
                Optitronic2State::getInstance().storeSingleRegister(write.registerAddress, write.value);
            }
            mPendingReadRequest = false;
            break;
        }

        case FC_WRITE_MULTIPLE_REGISTERS:
        {
            if (frame.dataLength > 5)
            {
                ModbusWriteMultiple write;
                if (parseWriteMultiple(frame, write))
                {
                    Optitronic2State::getInstance().storeRegisters(
                            write.startRegister, write.values, write.quantity);
                }
            }
            mPendingReadRequest = false;
            break;
        }

        default:
            break;
    }
}

void ModbusRelayTask::injectPendingWrites()
{
    uint16_t reg;
    uint16_t value;

    if (!optitronic2MqttTask.hasPendingWrite())
    {
        return;
    }

    if (!optitronic2MqttTask.dequeuePendingWrite(reg, value))
    {
        return;
    }

    // Build a Modbus write single register frame (fn=0x06) as master → slave
    uint8_t frame[8];
    frame[0] = MODBUS_SLAVE_ADDRESS;
    frame[1] = FC_WRITE_SINGLE_REGISTER;
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;

    // Calculate CRC-16/Modbus
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < 6; i++)
    {
        crc ^= frame[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    // === ONE-WIRE UART: Connect TX signal to both pins before transmitting ===
    int txSignal = uart_periph_signal[2].pins[SOC_UART_TX_PIN_IDX].signal;
    int rxSignal = uart_periph_signal[2].pins[SOC_UART_RX_PIN_IDX].signal;

    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_TX, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(config::GPIO_MAIN_TX, txSignal, false, false);
    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_RX, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(config::GPIO_MAIN_RX, txSignal, false, false);
    delayMicroseconds(10);

    // Ensure inter-frame silence before injecting
    vTaskDelay(pdMS_TO_TICKS(3));

    // Enable translator transmit direction
    digitalWrite(config::GPIO_ENABLE_TX_MAIN, HIGH);

    // Send write frame
    Serial2.write(frame, 8);
    Serial2.flush();

    // Back to receive mode
    digitalWrite(config::GPIO_ENABLE_TX_MAIN, LOW);

    // Disconnect TX pins, restore RX input
    esp_rom_gpio_connect_out_signal(config::GPIO_MAIN_TX, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_TX, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_out_signal(config::GPIO_MAIN_RX, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)config::GPIO_MAIN_RX, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(config::GPIO_MAIN_RX, rxSignal, false);

    // Clear echo bytes
    delayMicroseconds(200);
    while (Serial2.available()) { Serial2.read(); mEchoBytes++; }

    mWritesInjected++;

    Serial.printf("[relay] WRITE reg=0x%04X val=0x%04X\n", reg, value);

    // Wait for response from slave (up to 50ms)
    unsigned long start = millis();
    while ((millis() - start) < 50)
    {
        if (Serial2.available())
        {
            uint8_t respBuf[16];
            uint8_t respLen = 0;
            unsigned long lastByte = millis();
            while ((millis() - lastByte) < MODBUS_FRAME_SILENCE_MS && respLen < 16)
            {
                if (Serial2.available())
                {
                    respBuf[respLen++] = Serial2.read();
                    lastByte = millis();
                }
            }
            if (respLen >= 5)
            {
                parseFrame(respBuf, respLen, false);
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace aquamqtt
