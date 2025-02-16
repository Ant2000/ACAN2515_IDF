#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <ACAN2515.h>
#include <esp_log.h>

#define MCP2515_CS  static_cast<gpio_num_t>(19)
#define MCP2515_INT static_cast<gpio_num_t>(34)
#define SCK_PIN     static_cast<gpio_num_t>(26)
#define MISO_PIN    static_cast<gpio_num_t>(21)
#define MOSI_PIN    static_cast<gpio_num_t>(22)

#define SPI_HOST VSPI_HOST
#define QUARTZ_FREQUENCY (8UL * 1000UL * 1000UL) // 8 MHz Quartz

spi_device_handle_t spiDevice;
ACAN2515 can(nullptr, MCP2515_INT);

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

extern "C" [[noreturn]] void app_main()
{
    constexpr spi_bus_config_t bus_config = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .max_transfer_sz = 128,
    };

    constexpr spi_device_interface_config_t dev_config = {
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 2 * 1000 * 1000,
        .spics_io_num = MCP2515_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI_HOST, &dev_config, &spiDevice);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("MAIN", "Configure ACAN2515");
    can.setDeviceHandle(spiDevice);
    ACAN2515Settings settings (QUARTZ_FREQUENCY, 500UL * 1000UL) ; // CAN bit rate 500 kb/s
    settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select normal mode
    const uint16_t errorCode = can.begin (settings, [](void *params) { can.isr () ; }) ;
    if (errorCode == 0) {
        ESP_LOGI("MAIN","Bit Rate pre-scaler: %d", settings.mBitRatePrescaler) ;
        ESP_LOGI("MAIN","Propagation Segment: %d", settings.mPropagationSegment) ;
        ESP_LOGI("MAIN","Phase segment 1: %d", settings.mPhaseSegment1) ;
        ESP_LOGI("MAIN","Phase segment 2: %d", settings.mPhaseSegment2) ;
        ESP_LOGI("MAIN","SJW: %d", settings.mSJW) ;
        ESP_LOGI("MAIN","Triple Sampling: %s", settings.mTripleSampling ? "yes" : "no") ;
        ESP_LOGI("MAIN","Actual bit rate: %lu bit/s", settings.actualBitRate ());
        ESP_LOGI("MAIN","Exact bit rate ? %s", settings.exactBitRate () ? "yes" : "no") ;
        ESP_LOGI("MAIN","Sample point: %lu %%", settings.samplePointFromBitStart ());
    } else{
        ESP_LOGI("MAIN","Configuration error 0x%02X", errorCode) ;
    }

    while (true) {
        CANMessage frame  ;
        frame.id = 0x123 ;
        frame.len = 4 ;
        frame.data [0] = 0x11 ;
        frame.data [1] = 0x22 ;
        frame.data [2] = 0x33 ;
        frame.data [3] = 0x44 ;

        if (gBlinkLedDate < esp_timer_get_time()) {
            gBlinkLedDate += 2000000 ;
            // digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
            const bool ok = can.tryToSend (frame) ;
            if (ok) {
                gSentFrameCount += 1 ;
                ESP_LOGI("MAIN","Sent: %lu", gSentFrameCount) ;
            }else{
                ESP_LOGI("MAIN","Send failure") ;
            }
        }
        if (can.available ()) {
            can.receive (frame) ;
            gReceivedFrameCount ++ ;
            ESP_LOGI("MAIN","%lX, %d %d %d %d %d %d %d %d\n", frame.id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            ESP_LOGI("MAIN","Received: %lu", gReceivedFrameCount) ;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
