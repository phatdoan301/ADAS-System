#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>
#include <SPI.h>

//CAN
#include <driver/twai.h>

//LVGL + GUI
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "ui.h"

//Bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//BUzzer 
#define BUZZER_PIN 42
//TWAI (CAN)
#define CAN_TX_PIN GPIO_NUM_4  // Chân TX (có thể thay đổi)
#define CAN_RX_PIN GPIO_NUM_5  // Chân RX (có thể thay đổi)
//TFT và LVGL
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;

TFT_eSPI lcd = TFT_eSPI(); /* TFT entity */

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[ screenWidth * screenHeight / 10 ];

//Blind_spot distance
uint8_t blind_detect[2];
int i = 0;
//Khoang cach vat can
volatile uint8_t obstacle_dist = 0;
//Trang thai lan duong
uint8_t lane_status;
//Phanh khan cap
// TTC (Time to Collision)
#define TTC_THRESHOLD 8.0 // Ngưỡng TTC an toàn (giây)
volatile float ttc = INFINITY; // Biến lưu giá trị TTC
bool blockJoystick = false; // Trạng thái chặn joystick
unsigned long blockJoystickStartTime = 0; // Thời gian bắt đầu chặn joystick
bool continuousBuzzer = false; // Trạng thái kêu liên tục của buzzer
//Motor data
const int DEAD_ZONE = 200;      // Kích thước vùng chết (điều chỉnh tùy ý)
const int CENTER = 512;        // Giá trị trung tâm 
int motor_left,motor_right;
volatile uint8_t speed = 0;

//Bluetooth
QueueHandle_t joystickQueue; // Hàng đợi FreeRTOS để truyền dữ liệu
BLEServer *pServer = NULL;
bool deviceConnected = false;
bool connectedBuzzer = false;
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks 
{
  void onConnect(BLEServer *pServer) 
  {
    Serial.println("Device connected");
    deviceConnected = true;
    connectedBuzzer = true;
  };
  void onDisconnect(BLEServer *pServer) 
  {
    deviceConnected = false;
    Serial.println("Device disconnected, waiting for new connection...");
    delay(500);
    pServer->startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks 
{
  void onWrite(BLECharacteristic *pCharacteristic) 
  {
    String rxValue = pCharacteristic->getValue();
    String id,value;
    bool id_check = false;
    bool val_check = false;
    if (rxValue.length() > 0) 
    {
      for (int i = 0; i < rxValue.length(); i++) 
      {
        if (rxValue[i] == 1) 
        {
          id_check = true;
          continue;
        } 
        else if (rxValue[i] == 2) 
        {
          id_check = false;
          val_check = true;
          continue;
        } 
        else if (rxValue[i] == 3) 
        {
          val_check = false;
          continue;
        }
        if (id_check) id += rxValue[i];
        else if (val_check) value += rxValue[i];
      }
      char buffer[128];
      strncpy(buffer, value.c_str(), sizeof(buffer) - 1);
      buffer[sizeof(buffer) - 1] = '\0'; // Đảm bảo chuỗi kết thúc null
      if (xQueueSend(joystickQueue, (void *)buffer, pdMS_TO_TICKS(100)) != pdPASS)
      {
        Serial.println("Queue is full. Failed to enqueue data.");
        xQueueReset(joystickQueue);
        xQueueSend(joystickQueue, (void *)buffer, pdMS_TO_TICKS(100));
      }
    }
  }
};
//_______________________
/* display flash */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  lcd.startWrite();
  lcd.setAddrWindow( area->x1, area->y1, w, h );
  lcd.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  lcd.endWrite();

  lv_disp_flush_ready( disp );
}

//tính TTC
float calculateTTC(uint8_t distance, uint8_t speed) 
{
  if (speed == 0 || distance == 0)
  {
    return INFINITY; // Tránh chia cho 0 hoặc không có vật cản
  }
  return (float)distance / (float)speed; // TTC = khoảng cách / tốc độ
}
//Tasks
void TTC_task(void *pV)
{
  bool TTCencountered = false;
  while (1) 
  {
    // Tính TTC
    ttc = calculateTTC(obstacle_dist, speed);
    if (obstacle_dist > 30 && ttc != INFINITY)
    {
      TTCencountered = false;
    }
    if (ttc <= TTC_THRESHOLD && ttc != INFINITY && TTCencountered == false)
    {
      blockJoystick = true; // Chặn joystick
      continuousBuzzer = true;
      for (int i = 0; i < 5; i++) 
      {  // Send 5 times
        twai_message_t tx_msg;
        tx_msg.identifier = 0x01;
        tx_msg.extd = 0;          
        tx_msg.rtr = 0;           
        tx_msg.data_length_code = 2;
        tx_msg.data[0] = 0;       // Stop command
        tx_msg.data[1] = 0;       // Stop command
        if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) 
        {
          Serial.println("Stop command transmitted");
        } 
        else 
        {
          Serial.println("Failed to transmit stop command");
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Small delay between messages
      }
      TTCencountered = true;
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      blockJoystick = false; // Bỏ chặn joystick
      continuousBuzzer = false; // Tắt buzzer liên tục
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Kiểm tra TTC mỗi 50ms
  }
}
void screen_task(void *pV)
{
  uint8_t warning_state=0;
  while(1)
  {
    lv_timer_handler();
    //Toc do dong co
    lv_label_set_text_fmt(ui_Label1, "%d", speed);
    if(deviceConnected)
    {
      lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_HIDDEN);
    }
    else 
    {
      lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_HIDDEN);
    }
    if(blind_detect[0] == 1)
    {
      lv_obj_clear_flag(ui_Image4, LV_OBJ_FLAG_HIDDEN);
    }
    else lv_obj_add_flag(ui_Image4, LV_OBJ_FLAG_HIDDEN);
    if(blind_detect[1] == 1)
    {
      lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_HIDDEN);
    }
    else lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_HIDDEN);
    if(obstacle_dist != 0)
    {
      lv_obj_clear_flag(ui_Image6, LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text_fmt(ui_Label3, "%d cm", obstacle_dist);
      if(obstacle_dist <= 50)
      {
        if(obstacle_dist <= 30)
        {
          warning_state = 1;
        }
        else warning_state ^= 1;
      }
      else warning_state = 0;
      if(warning_state == 1)
      {
        lv_obj_clear_flag(ui_Image11, LV_OBJ_FLAG_HIDDEN);
      }
      else if(warning_state == 0)
      {
        lv_obj_add_flag(ui_Image11, LV_OBJ_FLAG_HIDDEN);
      }
    }
    else
    {
      lv_obj_add_flag(ui_Image6, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Image11, LV_OBJ_FLAG_HIDDEN);
      lv_label_set_text_fmt(ui_Label3, "", obstacle_dist);
    }
    if(lane_status == 0)
    {
      lv_obj_add_flag(ui_Image12, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Image13, LV_OBJ_FLAG_HIDDEN);
    }
    else if(lane_status == 1)
    {
      lv_obj_clear_flag(ui_Image12, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Image13, LV_OBJ_FLAG_HIDDEN);
    }
    else if(lane_status == 2)
    {
      lv_obj_add_flag(ui_Image12, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(ui_Image13, LV_OBJ_FLAG_HIDDEN);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void CAN_task(void *pV)
{
  while(1)
  {
    twai_message_t rx_message;
    if (twai_receive(&rx_message, pdMS_TO_TICKS(1000)) == ESP_OK) 
    {
      //Serial.print("Received ID: 0x");
      //Serial.print(rx_message.identifier, HEX);
      if(rx_message.identifier == 2)
      {
        lane_status = rx_message.data[0];
        obstacle_dist = rx_message.data[1];
      }
      if(rx_message.identifier == 3)
      {
        blind_detect[0] = rx_message.data[0];
        blind_detect[1] = rx_message.data[1];
      }
      if(rx_message.identifier == 4)
      {
        speed = rx_message.data[0];
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    //Serial.println(i);
    //i++;
  }
}
void processjoystickData(void *pV)
{
  char rxData[32]; // Bộ đệm để nhận dữ liệu từ hàng đợi
  int x,y,joystick_x,joystick_y;
  while(1)
  {
    // Chờ dữ liệu từ hàng đợi
    if (xQueueReceive(joystickQueue, &rxData, portMAX_DELAY) == pdPASS) 
    {
      if (blockJoystick) 
      {
        twai_message_t tx_msg;
        tx_msg.identifier = 0x01; 
        tx_msg.extd = 0;          
        tx_msg.rtr = 0;           
        tx_msg.data_length_code = 2;
        tx_msg.data[0] = 0;  // Stop
        tx_msg.data[1] = 0;  // Stop
        
        // Send multiple times for reliability
        for (int i = 0; i < 5; i++) 
        {
          if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) 
          {
            Serial.println("Emergency stop command transmitted");
          }
          vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        continue;
      }
      sscanf(rxData, "%d,%d", &joystick_x, &joystick_y);
      // Xử lý Dead Zone cho trục Y
      if (abs(joystick_x - CENTER) < DEAD_ZONE) 
      {
        joystick_x = CENTER; // Đưa về giá trị trung tâm
      }
      // Xử lý Dead Zone cho trục Y
      if (abs(joystick_y - CENTER) < DEAD_ZONE) {
        joystick_y = CENTER; // Đưa về giá trị trung tâm
      }
      x = (joystick_x - 512) * 75 / 512;
      y = (joystick_y - 512) * 75 / 512;
      motor_left = y + x;
      motor_right = y - x;
      if(motor_left > 75) motor_left = 75;
      if(motor_left < -75) motor_left = -75;
      if(motor_right > 75) motor_right = 75;
      if(motor_right < -75) motor_right = -75;
      Serial.printf("Motor left: %d, Motor right: %d\n", motor_left, motor_right);
      twai_message_t tx_msg;
      tx_msg.identifier = 0x01; 
      tx_msg.extd = 0;          
      tx_msg.rtr = 0;           
      tx_msg.data_length_code = 2;
      tx_msg.data[0] = motor_left;
      tx_msg.data[1] = motor_right;
      if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) == ESP_OK) 
      {
        Serial.println("Motor speed transmitted");
      } 
      else 
      {
        Serial.println("Failed to transmit Motor speed");
      }
    }
  }
}
void buzzerTask(void *pV)
{
  uint8_t buzzer_state = 0;
  int buzzer_interval;
  while(1)
  {
    if(connectedBuzzer)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      connectedBuzzer = false;
    }
    if (continuousBuzzer) 
    {
      digitalWrite(BUZZER_PIN, HIGH); // Buzzer kêu liên tục
      vTaskDelay(10 / portTICK_PERIOD_MS); // Giữ trạng thái bật, kiểm tra lại sau 10ms
      continue;
    }
    else digitalWrite(BUZZER_PIN, LOW);
    if(obstacle_dist != 0)
    {
      if(obstacle_dist <= 50) 
      {
        buzzer_state = 1;
        buzzer_interval = 1000;
      }
      if(obstacle_dist <= 30)
      {
        buzzer_state = 1;
        buzzer_interval = 500;
      }
      if(obstacle_dist <= 20)
      {
        buzzer_state = 1;
        buzzer_interval = 200;
      }
    }
    else buzzer_state = 0;
    if(buzzer_state == 1)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(buzzer_interval / portTICK_PERIOD_MS);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(buzzer_interval / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS); // Tắt buzzer và chờ 10ms
    }
  }
}
void setup()
{
  Serial.begin( 115200 ); /*serial init */
  pinMode(BUZZER_PIN, OUTPUT);

  //Cấu hình LVGL
  lcd.begin();          
  lcd.setRotation(2); 
  lcd.fillScreen(TFT_BLACK);
  delay(100);

  lv_init();
  lv_disp_draw_buf_init( &draw_buf, buf1, NULL, screenWidth * screenHeight / 10 );
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  lcd.fillScreen(TFT_BLACK);
  ui_init();

  //Queue
  joystickQueue = xQueueCreate(10, sizeof(char) * 32); // 10 mục, mỗi mục 128 ký tự

  //Cấu hình BLE
  BLEDevice::init("ADAS_System");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a device to connect...");

  // Cấu hình TWAI
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // Tốc độ 500kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Nhận tất cả tin nhắn

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) 
  {
    Serial.println("TWAI Driver Installed");
  } 
  else 
  {
    Serial.println("Failed to Install TWAI Driver");
    return;
  }
  
  if (twai_start() == ESP_OK) 
  {
    Serial.println("TWAI Started");
  } 
  else 
  {
    Serial.println("Failed to Start TWAI");
    return;
  }

  //Tạo task
  xTaskCreate(screen_task,"Disp_task", 4096, NULL, 16, NULL);
  xTaskCreate(CAN_task, "CAN_task", 2048, NULL, 16, NULL);
  xTaskCreate(processjoystickData, "BLE joystick task", 2048, NULL, 16, NULL);
  xTaskCreate(buzzerTask, "Buzzer task", 1024, NULL, 16, NULL);
  xTaskCreate(TTC_task, "TTC_task", 2048, NULL, 18, NULL);
}
void loop()
{

}