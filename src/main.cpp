#include "FreeRTOS.h" /* Must come first. */
#include "task.h"

#include <stdio.h>
#include <fmt/base.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "cbsd/cbsd.h"
#include "cblora/cblora.h"

#include "BME280Sensor/BME280Sensor.h"

#include "VEML7700.h"

#include "DS3231.h"
// #include <ctime>
// #include <pico/util/datetime.h>

#include "btstack_memory.h"
#include "hci.h"
#include "btstack_config.h"
#include "btstack_event.h"
#include "btstack.h"
#include "temp_sensor.h"

#include "hardware/uart.h"

#include "hardware/irq.h"
#include "hardware/spi.h"
#include <semphr.h>

// If this is defined, LoRa is disabled to allow printf to use UART0
//#define DEBUG

#define TEST_TASK_PRIORITY ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )

// Standard Task priority
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

TaskHandle_t ledTaskHandle;
TaskHandle_t sdTaskHandle;
TaskHandle_t printTaskHandle;
TaskHandle_t btTaskHandle;
TaskHandle_t windDirectionTaskHandle;
TaskHandle_t windSpeedTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t bme280TaskHandle;
TaskHandle_t ds3231TaskHandle;
TaskHandle_t veml7700TaskHandle;

QueueHandle_t btstack_event_queue;

std::string weatherStationData;
std::string dateTime;
std::string gpsData;
float luxValue;

SemaphoreHandle_t i2c_mutex;

void main_task(__unused void *params);
int picow_bt_example_init(void);
void picow_bt_example_main(void);
// static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
uint16_t attReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
int attWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
void heartbeatHandler(btstack_timer_source_t *ts);

void ledTask(void *params);




#define SPI_PORT spi0
#define CS_PIN 17   // GP17 22
#define CLK_PIN 18  // GP18 24
#define MOSI_PIN 19 // GP19 25
#define MISO_PIN 16 // GP16 21
const uint LED_PIN = 25;
const uint RAIN_INTERRUPT_PIN = 14;
const uint ANEMOMETER_INTERRUPT_PIN = 15;
const bool CALLBACK_ENABLED = true;
bool volatile ledShouldBeOn = false;
uint volatile buttonPresses = 0;
uint volatile anemometerClicks = 0;
int volatile windSpeed = 0;
char windDirectionName[4] = "NNE";
float temperature, pressure, humidity;
int windDirectionADCValue;

int calculateWindSpeed();
void getDirectionFromADCValue(int adcValue, char* buffer);
void printDirectionFromADCValue(int adcValue);

void windDirectionTask(void* pvParameters);
void gpsTask(__unused void* pvParameters);

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

void weatherSensorsCallback(uint gpio, __unused uint32_t events) {
  if (gpio == RAIN_INTERRUPT_PIN) {
    ledShouldBeOn = !ledShouldBeOn;
    buttonPresses++;
  }
  else if (gpio == ANEMOMETER_INTERRUPT_PIN) {
    // printf("---------------------------------- ANEMOMETER_INTERRUPT_PIN\n");
    anemometerClicks++;
  }
}

bool timerCallback(__unused repeating_timer_t* repeatingTimerInfo) {
    //printf("anemometerClicks %d ", anemometerClicks);
    windSpeed = calculateWindSpeed();
    // printf("Rain: %d\n", buttonPresses);
    // printf("Wind Speed: %d\n", windSpeed);
    // printf("Wind Direction: %s\n", windDirectionName);
    //printf("windSpeed %dmph\n", windSpeed);
    return true;
}

int calculateWindSpeed() {
    // One anemometer turn in one second is 1.492 miles per hour
    long speed = (14920 * anemometerClicks) / 10000;
    anemometerClicks = 0;
    return (int)speed;
}


void getDirectionFromADCValue(int adcValue, char* buffer) {
  windDirectionADCValue = adcValue;
    // 7% with corrected min/max to prevent overlap
    if ((adcValue >= 730) && (adcValue <= 740)) strcpy(buffer, "N");   // 736
    if ((adcValue >= 374) && (adcValue <= 438)) strcpy(buffer, "NNE"); // 397
    if ((adcValue >= 425) && (adcValue <= 499)) strcpy(buffer, "NE");  // 449
    if ((adcValue >= 77) && (adcValue <= 91)) strcpy(buffer, "ENE");   // 85
    if ((adcValue >= 86) && (adcValue <= 100)) strcpy(buffer, "E");    // 95
    if ((adcValue >= 63) && (adcValue <= 73)) strcpy(buffer, "ESE");   // 67
    if ((adcValue >= 171) && (adcValue <= 201)) strcpy(buffer, "SE");  // 186
    if ((adcValue >= 117) && (adcValue <= 137)) strcpy(buffer, "SSE"); // 128
    if ((adcValue >= 265) && (adcValue <= 311)) strcpy(buffer, "S");   // 286
    if ((adcValue >= 225) && (adcValue <= 265)) strcpy(buffer, "SSW"); // 244
    if ((adcValue >= 590) && (adcValue <= 600)) strcpy(buffer, "SW");  // 598
    if ((adcValue >= 570) && (adcValue <= 580)) strcpy(buffer, "WSW"); // 572
    if ((adcValue >= 870) && (adcValue <= 890)) strcpy(buffer, "W");  // 873
    if ((adcValue >= 770) && (adcValue <= 780)) strcpy(buffer, "WNW"); // 772
    if ((adcValue >= 820) && (adcValue <= 830)) strcpy(buffer, "NW");  // 823
    if ((adcValue >= 660) && (adcValue <= 670)) strcpy(buffer, "NNW"); // 663
}





#define APP_AD_FLAGS 0x06
static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x17, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);

#define HEARTBEAT_PERIOD_MS 1000

// static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;
hci_con_handle_t con_handle;
uint16_t current_temp = 222;
int le_notification_enabled;



void handlePacket(__unused uint8_t packet_type, uint16_t channel,
    uint8_t* packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);

    bd_addr_t local_addr;
    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type) {
        case BTSTACK_EVENT_STATE: {
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // ble limitation
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);
            break;
        }
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = 0;
            break;
        case ATT_EVENT_CAN_SEND_NOW:
        printf("sending data...\n");
            att_server_notify(con_handle,
              ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE,
              (uint8_t*)&current_temp,
              sizeof(current_temp));
            break;
        default:
            break;
    }
}

uint16_t attReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE){
        return att_read_callback_handle_blob((const uint8_t *)&current_temp, sizeof(current_temp), offset, buffer, buffer_size);
    }
    return 0;
}

int attWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    
    if (att_handle != ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) return 0;
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    con_handle = connection_handle;
    if (le_notification_enabled) {
      // unsigned long ulVar = 10UL;
      // if (btstack_event_queue != 0) {
      //   printf ("incoming request...\n");
        // THIS FREEZES WHEN THE CLIENT CONNECTS:
      //   xQueueSend(btstack_event_queue, (void *)&ulVar, (TickType_t)10);
      //   printf("sent in response to incoming request\n");
      // }

      // THIS FREEZES WHEN THE CLIENT CONNECTS:
      // if (con_handle != 0) {
      //   att_server_request_can_send_now_event(con_handle);
      // }
    }
    return 0;
}



void vLaunch(void) {
  //btstack_event_queue = xQueueCreate(10, sizeof(unsigned long));
  xTaskCreate(main_task, "TestMainThread", 1024, NULL, TASK_PRIORITY, &btTaskHandle);
  xTaskCreate(ledTask, "LEDTask", 1024, NULL, tskIDLE_PRIORITY, &ledTaskHandle);
  vTaskStartScheduler();
}
void main_task(__unused void *params) {
  int res = picow_bt_example_init();
  if (res){
    printf("sdfdsfsdfsdfdfsdfsdfsdfsdf\n");
      return;
  }
  while(true) {
    unsigned long event;
    if (btstack_event_queue != 0) {
      printf("receiving data from queue...\n");
      if (xQueueReceive(btstack_event_queue, &event, portMAX_DELAY)) {
        printf("requesting to send...\n");
        att_server_request_can_send_now_event(con_handle);
      }    
    }
    else {
      printf("NULL queue\n");
    }
    vTaskDelay(5000);
  }
}
int picow_bt_example_init(void) {
    l2cap_init();
    sm_init();
    att_server_init(profile_data, attReadCallback, attWriteCallback);
    hci_event_callback_registration.callback = &handlePacket;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(handlePacket);
    hci_power_control(HCI_POWER_ON);

  return 0;
}



void ledTask(__unused void *params) {
  while (true) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    vTaskDelay(1000);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    vTaskDelay(1000);

    // unsigned long ulVar = 10UL;
    // if (btstack_event_queue != 0) {
    //   printf ("sending\n");
    //   xQueueSend(btstack_event_queue, (void *)&ulVar, (TickType_t)10);
    //   printf("sent\n");
    // }

    printf("ledTask\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    vTaskDelay(1000);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    vTaskDelay(1000);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(
      printTaskHandle,
      (uint32_t)666,
      eSetValueWithOverwrite, // or eSetValueWithoutOverwrite
      &xHigherPriorityTaskWoken
    );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void printTask(__unused void* pvParameters) {
    uint32_t receivedValue;
    for (;;) {
        xTaskNotifyWait(0, 0, &receivedValue, portMAX_DELAY);
        uint32_t data = receivedValue;
        printf("data = %ld\n", data);
    }
}



void windSpeedTask(__unused void* pvParameters) {
  while(true) {
    windSpeed = calculateWindSpeed();
    vTaskDelay(500);
  }
}

SemaphoreHandle_t uart_mutex;
void windDirectionTask(__unused void* pvParameters) {
  Codebrane::CBSD* pCBSD = static_cast<Codebrane::CBSD*>(pvParameters);
  pCBSD->init();
  // pCBSD->mount();
  // pCBSD->unmount();

  // Codebrane::CBLora cbLora;
  // cbLora.setupLora(433, 10, "CODEBRANE");

  spi_init(SPI_PORT, 1350000);
  spi_set_format(SPI_PORT, 8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST);
  gpio_set_function(CLK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(CS_PIN, GPIO_FUNC_SPI);
  gpio_init(CS_PIN);
  gpio_set_dir(CS_PIN, GPIO_OUT);
  gpio_put(CS_PIN, 1);

  int chan = 0;
  uint8_t buffer[3];
  buffer[0] = 1;
  buffer[1] = (8 + chan) << 4;
  buffer[2] = 0;

  //unsigned char loraBuffer[35] = "Hello From Pico!";
//  unsigned char loraBuffer[255] = "1,2,3.4,534,/sdf,43234/,dfg,rwe,234,fwef,,sdfesrwerse,,fse,V";

  while(true) {
    cs_select();
    sleep_ms(10);

    uint8_t returnData[10];
    spi_write_read_blocking(SPI_PORT, buffer, returnData, sizeof(buffer));
    int data = ( (returnData[1]&3) << 8 ) | returnData[2];
    printf("WIND DIRECTION DATA = %d\n", data);
    cs_deselect();
    sleep_ms(10);

    getDirectionFromADCValue(data, windDirectionName);

    printf("%s %d **************** Rain: %d, Wind Speed: %d, Wind Direction: %s, Temp: %.2f, Press: %.2f, Humidity: %.2f, Lux: %.2f\n",
      dateTime.c_str(), windDirectionADCValue, buttonPresses, windSpeed, windDirectionName, temperature, pressure, humidity, luxValue);

    sleep_ms(100);
    
    // weatherStationData = fmt::format("{},{},{},{}",
    //   buttonPresses, windSpeed, windDirectionName, gpsData);
    weatherStationData = fmt::format("{},{},{},{},{},{},{},{},{},{}",
      dateTime.c_str(), luxValue, buttonPresses, windSpeed, windDirectionName,
      temperature, pressure, humidity, windDirectionADCValue, gpsData);

    //pCBSD->writeAfterInit();
    pCBSD->writeAfterInit(weatherStationData);
    
    // pCBSD->write(weatherStationData);

    //pCBSD->mount();
    //pCBSD->unmount();

    // unsigned char data[weatherStationData.size()];
    // std::copy(weatherStationData.begin(), weatherStationData.end(), data);

    //auto uchrs = reinterpret_cast<unsigned char*>(const_cast<char*>(weatherStationData.c_str()));
    //printf("weatherStationData.length() = %d\n",weatherStationData.length());
    //cbLora.sendLoraPacket(uchrs, weatherStationData.length(), 0);
    // cbLora.sendLoraPacket(uchrs, weatherStationData.length(), 0);
    // printf("SENDING LORA\n");
    // std::size_t length = weatherStationData.length();
    // unsigned char* loraBuffer = new unsigned char[weatherStationData.length()];
    // weatherStationData.copy(reinterpret_cast<char*>(loraBuffer), length, 0);
    // cbLora.sendLoraPacket(loraBuffer, sizeof(loraBuffer), 0);
    // printf("SENT LORA\n");

    printf("sending to lora...\n");
    const char *msg = "testing from the weather station pico\n";
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100))) {
      // uart_write_blocking(uart0, (const uint8_t *)msg, strlen(msg));
      #ifndef DEBUG
      uart_write_blocking(uart0, (const uint8_t *)weatherStationData.c_str(),
        strlen(weatherStationData.c_str()));
      #endif
      xSemaphoreGive(uart_mutex);
    }
    printf("sent to lora...\n");
    
    unsigned long ulVar = 10UL;
    if (btstack_event_queue != 0) {
      printf ("sending\n");
      xQueueSend(btstack_event_queue, (void *)&ulVar, (TickType_t)10);
      printf("sent\n");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}







// On receiving new valid GPS time, just overwrite current_time with GPS time
typedef struct {
    int year, month, day;
    int hour, minute, second;
} gps_time_t;
gps_time_t current_time;
// Simple helper to increment time by one second
void increment_time_by_one_second(gps_time_t *t) {
    t->second++;
    if (t->second >= 60) {
        t->second = 0;
        t->minute++;
        if (t->minute >= 60) {
            t->minute = 0;
            t->hour++;
            if (t->hour >= 24) {
                t->hour = 0;
                // Naive day increment; you can add proper month-length/leap year logic here
                t->day++;
                // For simplicity, just reset month/day to 1 if you want
                if (t->day > 31) {
                    t->day = 1;
                    t->month++;
                    if (t->month > 12) {
                        t->month = 1;
                        t->year++;
                    }
                }
            }
        }
    }
}
void time_keeper_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        increment_time_by_one_second(&current_time);
        printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
               current_time.year, current_time.month, current_time.day,
               current_time.hour, current_time.minute, current_time.second);
    }
}

/*
void time_increment_task(__unused void *pv) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1 second

        // Increment time by 1 second
        current_time.second++;
        if (current_time.second >= 60) {
            current_time.second = 0;
            current_time.minute++;
            if (current_time.minute >= 60) {
                current_time.minute = 0;
                current_time.hour++;
                if (current_time.hour >= 24) {
                    current_time.hour = 0;
                    // increment day, month, year accordingly
                    // handle month length and leap years here
                }
            }
        }
    }
}
    */


#define GPS_BUFFER_SIZE 256
static char gps_buffer[GPS_BUFFER_SIZE];
static volatile uint16_t gps_head = 0;
static volatile uint16_t gps_tail = 0;
void gps_buffer_put(char c) {
    uint16_t next = (gps_head + 1) % GPS_BUFFER_SIZE;
    if (next != gps_tail) {  // Avoid overwrite
        gps_buffer[gps_head] = c;
        gps_head = next;
    }
}
bool gps_buffer_get(char *c) {
    if (gps_tail == gps_head) {
        return false; // Buffer empty
    }
    *c = gps_buffer[gps_tail];
    gps_tail = (gps_tail + 1) % GPS_BUFFER_SIZE;
    return true;
}
#define GPS_RX_PIN 5
static int irq_count = 0;
void on_uart_rx() {
    irq_count++;
    while (uart_is_readable(uart1)) {
        char c = uart_getc(uart1);
        gps_buffer_put(c);
    }
}
void init_uart() {
    uart_init(uart1, 9600); // GPS baud rate
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    // Enable UART RX interrupt
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);  // RX only
}
void gpsTask(__unused void *pvParameters) {
    char line[128];
    size_t index = 0;
    char c;

    while (1) {
      printf("IRQ fired %d times\n", irq_count);
        while (gps_buffer_get(&c)) {
            if (c == '\n') {
                line[index++] = '!';
                line[index] = '\0';
                // Process full line
                // if (strstr(line, "$GPGGA") || strstr(line, "$GPRMC")) {
                    printf("%s\n", line); // Could parse here
                    gpsData = fmt::format("{}",line);
                // }
                index = 0;
            } else if (index < sizeof(line) - 1) {
                // don't add \r. It shows up as ^M and data is on next line
                if (c != '\r') {
                  line[index++] = c;
                }
            } else {
                index = 0; // overflow, discard
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void update_time_from_gps(int year, int month, int day, int hour, int minute, int second) {
    current_time.year = year;
    current_time.month = month;
    current_time.day = day;
    current_time.hour = hour;
    current_time.minute = minute;
    current_time.second = second;
}




DS3231 rtc(i2c0);
void rtc_task(void* pvParameters) {
    rtc.init();

    while (true) {
        struct tm time;
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
          if (rtc.readTime(time)) {
              // printf("Time: %02d:%02d:%02d Date: %04d/%02d/%02d\n",
              //       time.tm_hour, time.tm_min, time.tm_sec,
              //       time.tm_year + 1900, time.tm_mon + 1, time.tm_mday);
              printf("Date: %02d/%02d/%04d Time: %02d:%02d:%02d\n",
                time.tm_mday, time.tm_mon + 1, time.tm_year + 1900,
                time.tm_hour, time.tm_min, time.tm_sec);

              dateTime = fmt::format("{:02d}/{:02d}/{:4d} {:02d}:{:02d}:{:02d}",
                time.tm_mday, time.tm_mon + 1, time.tm_year + 1900,
                time.tm_hour, time.tm_min, time.tm_sec);      
          }
          else {
              printf("Failed to read time\n");
          }
          xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void rtc_setup_task(void* pvParameters) {
    struct tm buildTime = {
        .tm_sec = 0,
        .tm_min = 45,
        .tm_hour = 12,
        .tm_mday = 28,
        .tm_mon = 6 - 1,     // June (0-based)
        .tm_year = 2025 - 1900,
        .tm_wday = 6,        // Optional: Saturday
    };

    if (rtc.setTime(buildTime)) {
        printf("RTC time set to %04d-%02d-%02d %02d:%02d:%02d\n",
               buildTime.tm_year + 1900, buildTime.tm_mon + 1, buildTime.tm_mday,
               buildTime.tm_hour, buildTime.tm_min, buildTime.tm_sec);
    }
    else {
        printf("Failed to set RTC time\n");
    }

    vTaskDelete(NULL); // Self-terminate
}














void gpsTask2(__unused void* pvParameters) {
  // uart_init(uart1, 9600);
  // gpio_set_function(4, GPIO_FUNC_UART);
  // gpio_set_function(5, GPIO_FUNC_UART);
  // uart_set_baudrate(uart1, 9600);
  // uart_set_hw_flow(uart1, false, false);
  // uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
  // uart_set_fifo_enabled(uart1, false);

  // char nmeaSentence[100];
  // int nmeaSentenceLength = 0;

  char buf[256];
  int i = 0;

  while (true) {
    printf("gpsTask\n");

    if (uart_is_readable(uart1)) {
        char ch = uart_getc(uart1);
        if (ch == '\n' || i >= 256 - 1) {
            buf[i] = '\0';
            printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> GPS: %s\n", buf);
            i = 0;
        } else if (ch != '\r') {
            buf[i++] = ch;
        }
    }

    /*
    while (uart_is_readable(uart1))
    {
        uint8_t ch = uart_getc(uart1);

        if (ch == '$')
        {
            nmeaSentence[0] = ch;
            nmeaSentenceLength = 1;
        }
        // remove \r and replace \n with \0 to make it null terminated string
        else if ((nmeaSentenceLength > 0) && (ch != '\r'))
        {
            nmeaSentence[nmeaSentenceLength++] = ch;
            printf("00000000000000000000000000000000000000000000 %s", nmeaSentence);
            if (ch == '\n')
            {
                nmeaSentence[nmeaSentenceLength] = '\0';
                nmeaSentenceLength = 0;
                printf("%s", nmeaSentence);

                if (strncmp(nmeaSentence + 3, "GGA", 3) == 0)
                {
                    float utc_time, latitude, longitude, hdop, altitude, speed, course;
                    int lock, satellites, date;
                    char active, ns, ew, units, speedstring[16], coursestring[16];
                    if (sscanf(nmeaSentence + 7, "%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &latitude, &ns, &longitude, &ew, &lock, &satellites, &hdop, &altitude, &units) >= 1)
                    {
                        printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>> Lat: %f Lon: %f Alt: %f\n", latitude, longitude, altitude);
                    }
                }
            }
        }
        //vTaskDelay(pdMS_TO_TICKS(500));
    } // while (uart_is_readable(uart1))
     */

    vTaskDelay(pdMS_TO_TICKS(10));
  } // while (true) {
}



void bme280Task(void* pvParameters) {
  BME280Sensor sensor(i2c0, 0x76, 8, 9);
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    if (!sensor.init()) {
      printf("Failed to init BME280\n");
      xSemaphoreGive(i2c_mutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(i2c_mutex);
  }

  while (true) {
    // pass variables by reference
    printf("<<<<<<<<<<<<<<<<<<<<<< READING BME280\n");
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      if (sensor.readSensor(temperature, pressure, humidity)) {
          printf("T: %.2f°C, P: %.2f hPa, H: %.2f%%\n", temperature, pressure, humidity);
      }
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}




void veml7700Task(void *params) {
    VEML7700 sensor(i2c0, 0x10, 8, 9);

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      if (!sensor.begin()) {
          xSemaphoreGive(i2c_mutex);
          printf("VEML7700 init failed\n");
          vTaskDelete(NULL);
      }
      xSemaphoreGive(i2c_mutex);
    }

    while (true) {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
          if (sensor.readLux(luxValue)) {
              printf("Lux: %.2f\n", luxValue);
          }
          else {
              printf("Failed to read lux\n");
          }
          xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1s delay
    }
}




void initLoraUart() {
  uart_init(uart0, 115200);
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);
}







int main(void) {
  #ifdef DEBUG
  stdio_init_all();
  #endif

  init_uart();

  #ifndef DEBUG
  initLoraUart();
  #endif

  // uart_init(uart1, 9600);
  // gpio_set_function(4, GPIO_FUNC_UART);
  // gpio_set_function(5, GPIO_FUNC_UART);
  // uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
  // uart_set_fifo_enabled(uart1, true);

  gpio_init(RAIN_INTERRUPT_PIN);
  gpio_set_dir(RAIN_INTERRUPT_PIN, GPIO_IN);
  gpio_pull_down(RAIN_INTERRUPT_PIN);

  gpio_init(ANEMOMETER_INTERRUPT_PIN);
  gpio_set_dir(ANEMOMETER_INTERRUPT_PIN, GPIO_IN);
  gpio_pull_down(ANEMOMETER_INTERRUPT_PIN);

  int _sda_pin = 8;
  int _scl_pin = 9;
  i2c_init(i2c0, 100 * 1000);
  gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(_sda_pin);
  gpio_pull_up(_scl_pin);

  // RAIN
  gpio_set_irq_enabled_with_callback(RAIN_INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE,
      CALLBACK_ENABLED, weatherSensorsCallback);

  // ANEMOMETER
  gpio_set_irq_enabled(ANEMOMETER_INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE, CALLBACK_ENABLED);

  // WIND DIRECTION
  // spi_init(SPI_PORT, 1350000);
  // spi_set_format(SPI_PORT, 8, (spi_cpol_t)0, (spi_cpha_t)0, SPI_MSB_FIRST);
  // gpio_set_function(CLK_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(CS_PIN, GPIO_FUNC_SPI);
  // gpio_init(CS_PIN);
  // gpio_set_dir(CS_PIN, GPIO_OUT);
  // gpio_put(CS_PIN, 1);

  Codebrane::CBSD cbsd;
  xTaskCreate(windDirectionTask, "WindDirectionTask", 4096, (void*)&cbsd, TASK_PRIORITY, &windDirectionTaskHandle);
  xTaskCreate(windSpeedTask, "WindSpeedTask", 1024, NULL, TASK_PRIORITY, &windSpeedTaskHandle);



  btstack_event_queue = xQueueCreate(10, sizeof(unsigned long));

  if (cyw43_arch_init()) {
    printf("failed to initialise cyw43_arch\n");
    return -1;
  }

  //vLaunch();





  // xTaskCreate(ledTask, "LEDTask", 256, NULL, TASK_PRIORITY, &ledTaskHandle);

//  Codebrane::CBSD cbsd;
//   xTaskCreate(
//     writeToSDTask,
//     "WriteToSDTask",
//     1024U,
//     (void*)&cbsd,
//     TASK_PRIORITY,
//     &sdTaskHandle);

  xTaskCreate(printTask, "PrintTask", 256, NULL, 2, &printTaskHandle);

  xTaskCreate(gpsTask, "GPSTask", 1024, NULL, 1, &gpsTaskHandle);

  xTaskCreate(bme280Task, "BME280Task", 1024, NULL, 1, &bme280TaskHandle);

  xTaskCreate(veml7700Task, "VEML7700Task", 1024, NULL, 1, &veml7700TaskHandle);

  /*
   set the rtc date/time
   ---------------------
   https://forum.arduino.cc/t/setting-accurate-time-with-ds3231-h-library/627993/7
   uncomment xTaskCreate/rtc_setup_task below.
   set the date/time in rtc_setup_task, e.g. 12:40:00.
   make and upload to the pico several minutes before 12:40:00.
   unplug the pico.
   at 12:40:00, plug in the pico.
   unplug the pico and plug in with bootsel held down for programming.
   comment out xTaskCreate/rtc_setup_task below.
   make and upload to the pico.
   the rtc will stay on the correct time if the battery is in place.
  */
  // xTaskCreate(
  //   rtc_setup_task,
  //   "RTC Setup",
  //   1024,
  //   nullptr,
  //   tskIDLE_PRIORITY + 2, // higher priority to run before other tasks
  //   nullptr
  // ); 
  xTaskCreate(
    rtc_task,
    "RTC Task",
    1024,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr
  );

  uart_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  vLaunch();
  
  // This only returns if something breaks so can't create any more tasks from the main thread.
  // Need to spawn them from already spawned tasks.
  // vTaskStartScheduler();

  // Should never get here
  while(1) {};
}
