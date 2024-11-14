#include "stm32f4xx_hal.h"  // Ensure you include the right header file for your MCU family

#define SerialPort Serial
uint8_t dataToSend = 0xCA;
uint8_t receivedData = 0x00;  // Make receivedData a global variable
uint8_t crcToSend;  // Global variable to store 8-bit CRC
uint8_t array_data[]={0X00,0X01,0X02,0X03,0X04,0X05,0X06,0X07,0X08,0X09,0X0A,0X0B,0XC,0X0D,0X0E,0X0F,0X10,0X11,0X12,0X13,0X14,0X15,0X16,0X17,0X18,0X19,0X1A,0X1B,0X1C,0X1D,0X1E,0X1F};
uint8_t array_index=0;
// Declare the SPI handle
int pressed_once = 0;
int counter_crc=0;
uint count_data_send;



 uint8_t MSB_BIT=0;
uint8_t full_16_bit_sent=0;




#include <SPI.h>
#include <vl53l8cx.h>

#define SerialPort Serial

#define PWREN_PIN 9

#define SPI_CLK_PIN 3
#define SPI_MISO_PIN  5
#define SPI_MOSI_PIN 4
#define CS_PIN 10


SPI_HandleTypeDef hspi1;



typedef struct {
    uint16_t distance_mm;
    uint16_t range_sigma_mm;
    uint8_t target_status;
} ZoneData;//I created this struct because , i downt to create three dimensional data with uint16_t usefull data[8][8][3] because status is uint*_t and i will be wasting memory 


ZoneData usefull_data[8][8][VL53L8CX_NB_TARGET_PER_ZONE];

SPIClass DEV_SPI(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);

void print_result(VL53L8CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

VL53L8CX sensor_vl53l8cx_top(&DEV_SPI, CS_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_8X8;

    uint8_t number_of_zones = res; // Either 16 or 64
    uint8_t zones_per_line = (number_of_zones == 16) ? 4 : 8;
char report[256];
uint8_t status;
uint8_t frequency_hz;










void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();  // Enable SPI1 clock
        __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock

        // SPI1 SCK, MISO, MOSI pin configuration
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1; // SPI1 Alternate function AF5
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // SPI1 NSS pin configuration (if used in hardware-controlled mode)
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



        // 
    }
}

void Init_SPI1(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
    hspi1.Init.CRCPolynomial = 7;  // This setting won't affect anything since CRCCalculation is disabled

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        SerialPort.println("SPI Initialization Error");
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        SerialPort.println("SPI Error Callback");
        SerialPort.print("SPI Error Code: ");
        SerialPort.println(hspi->ErrorCode, HEX);
    }
}

// Software implementation of CRC-8 calculation with polynomial 0x07
uint8_t calculateCRC8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;  // Initial value

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)//if it is 0 then false if it other then then it is true
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;//if the MSB was not 0 we shift by 1, we add a 0 then we don't neet to do a XOR with the  divideur 
        }
    }
    return crc;
}



void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable the GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PA0 and PA1 as output
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set as push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void setup()
{


  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
     SerialPort.println("PWREN HIGH");
    delay(10);
  }



    SerialPort.begin(9600);
    while (!SerialPort) {} // Wait for serial to be ready
    SerialPort.println("STM32 SPI Ready");
    HAL_Init();
    Init_SPI1();
    GPIO_Init();
             // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13 ,GPIO_PIN_SET);
              //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14 ,GPIO_PIN_SET);

    pinMode(PA13, INPUT); // It is LOW when you push it
    pinMode(PA14, INPUT); // It is LOW when you push it
    pinMode(PC13, INPUT); // It is LOW when you push it
    pinMode(PA1,INPUT);
    digitalWrite(PA4, HIGH);





// Initialize SPI bus.
  DEV_SPI.begin();

  // Configure VL53L8CX component.
  sensor_vl53l8cx_top.begin();

  pinMode(CS_PIN, OUTPUT); // Set CS pin as an output
  status=1;

  while(status!=0){
  status = sensor_vl53l8cx_top.init();
 // digitalWrite(CS_PIN, HIGH); // Force CS to low initially
  //delay(3000);
  SerialPort.println("Hey");
    SerialPort.println(status);
  }

  // Start Measurements
  
   SerialPort.println("Initializing");
   sensor_vl53l8cx_top.set_resolution(res);
//sensor_vl53l8cx_top.vl53l8cx_set_resolution(res);
  status = sensor_vl53l8cx_top.start_ranging();
  sensor_vl53l8cx_top.set_ranging_frequency_hz(60);
  sensor_vl53l8cx_top.get_ranging_frequency_hz(&frequency_hz);




}

void loop()
{


VL53L8CX_ResultsData Results;

 
if ((digitalRead(PC13) == GPIO_PIN_RESET) && pressed_once == 0){

  
  uint8_t NewDataReady = 0;
  do {
      status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
      SerialPort.println(status);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l8cx_top.get_ranging_data(&Results);
    //  print_result(&Results);
      sensor_vl53l8cx_top.get_ranging_frequency_hz(&frequency_hz);
      save_usefull_data(&Results);
      print_usefull_data();

  }
  count_data_send=0;
  uint8_t fixed_data;

  MSB_BIT=0;
uint8_t count_count=0;
  for (int row = 0; row < zones_per_line; row++) {
        // Print distances
    for (int col = 0; col < zones_per_line; col++) {
        for (int target = 0; target < VL53L8CX_NB_TARGET_PER_ZONE; target++) {
            
            uint16_t distance = usefull_data[row][col][target].distance_mm;
            uint8_t data_distance[2] = {distance >> 8, distance & 0xFF};  // MSB and LSB
            uint8_t sigma  = usefull_data[row][col][target].range_sigma_mm;
          // uint8_t sigma =count_count;
           //count_count++;
           SerialPort.println(count_count);
            // for (int byte = 0; byte < 2; byte++) {
              
                crcToSend = calculateCRC8(&sigma, 1);
                counter_crc=0;
                pressed_once = 1;
                uint8_t received_correctly_0, received_correctly_1;
              /*  SerialPort.println("Distance byte ");
                SerialPort.print(byte);
                SerialPort.print("= ");
                SerialPort.print(data_distance[byte]);
             */
              do {
                  counter_crc++;
                  crcToSend = calculateCRC8(&sigma, 1);  // Calculate CRC over the data
                  if (hspi1.State == HAL_SPI_STATE_BUSY)
                  {
                      SerialPort.println("Aborting previous SPI transmission");
                      if (HAL_SPI_Abort(&hspi1) != HAL_OK)
                      {
                          SerialPort.println("Error aborting SPI");
                      }
                  }

                  // Select the slave
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                // SerialPort.println("Starting SPI Transmission");

                  // Transmit data and CRC
                  if (HAL_SPI_Transmit(&hspi1, &sigma, 1, HAL_MAX_DELAY) == HAL_OK 
                  &&                HAL_SPI_Transmit(&hspi1, &crcToSend, 1, HAL_MAX_DELAY) == HAL_OK
                  )
                  {
                  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                    // SerialPort.println("Transmission Complete");
                  }
                  else
                  {
                      SerialPort.println("Error in SPI Transmission");
                      SerialPort.print("SPI Error Code: ");
                      SerialPort.println(hspi1.ErrorCode, HEX);
                  }

                  
                  // Read the received correctly signals from FPGA
                  received_correctly_0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13);
                  received_correctly_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14);
                 // SerialPort.println(crcToSend, HEX);
                  //SerialPort.println(counter_crc, DEC);

              } while (!(received_correctly_0 == GPIO_PIN_RESET && received_correctly_1 == GPIO_PIN_RESET));  // Check for '00' status


        // Deselect the slave
                  
              SerialPort.println("Transmission Done");
              // Reset button press flag to allow re-triggering
              pressed_once = 0;
              delay(250);
//             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

                  
           // }

        }

      }

    }       
      
        

  }
}

















void save_usefull_data(VL53L8CX_ResultsData *Result) {
    //uint8_t number_of_zones = res; // Either 16 or 64
    //uint8_t zones_per_line = (number_of_zones == 16) ? 4 : 8;

    // Define usefull_data as a 3D array: [number_of_zones][number_of_zones][VL53L8CX_NB_TARGET_PER_ZONE]

    // Loop over each zone
    for (int row = 0; row < zones_per_line; row++) {
        for (int col = 0; col < zones_per_line; col++) {
            int base_index = (row * zones_per_line) + col;

            // Loop over each potential target in the current zone
            for (int target = 0; target < VL53L8CX_NB_TARGET_PER_ZONE; target++) {
                int index = (VL53L8CX_NB_TARGET_PER_ZONE * base_index) + target;

                if (Result->nb_target_detected[base_index] > target) {
                    // Copy data to the array if the target is detected
                    usefull_data[row][col][target].distance_mm = Result->distance_mm[index];
                    usefull_data[row][col][target].range_sigma_mm = Result->range_sigma_mm[index];
                    usefull_data[row][col][target].target_status = Result->target_status[index];
                } else {
                    // Mark as invalid if no target is detected for this slot
                    usefull_data[row][col][target].distance_mm = -1;
                    usefull_data[row][col][target].range_sigma_mm = -1;
                    usefull_data[row][col][target].target_status = -1;
                }
            }
        }
    }
}
void print_usefull_data() {
   // uint8_t zones_per_line = (res == VL53L8CX_RESOLUTION_4X4) ? 4 : 8;

    for (int row = 0; row < zones_per_line; row++) {
        // Print distances
        for (int col = 0; col < zones_per_line; col++) {
            for (int target = 0; target < VL53L8CX_NB_TARGET_PER_ZONE; target++) {
                printf("| distance = %d mm ", usefull_data[row][col][target].distance_mm);
            }
        }
        printf("|\n");

        // Print sigmas
        for (int col = 0; col < zones_per_line; col++) {
            for (int target = 0; target < VL53L8CX_NB_TARGET_PER_ZONE; target++) {
                printf("| sigma = %d mm ", usefull_data[row][col][target].range_sigma_mm);
            }
        }
        printf("|\n");

        // Print statuses
        for (int col = 0; col < zones_per_line; col++) {
            for (int target = 0; target < VL53L8CX_NB_TARGET_PER_ZONE; target++) {
                printf("| status = %d ", usefull_data[row][col][target].target_status);
            }
        }
        printf("|\n");

        // Print a separator line after each row of zones
        for (int col = 0; col < zones_per_line; col++) {
            printf("--------------------");
        }
        printf("\n");
    }
}



void toggle_resolution(void)
{
  status = sensor_vl53l8cx_top.stop_ranging();

  switch (res) {
    case VL53L8CX_RESOLUTION_4X4:
      res = VL53L8CX_RESOLUTION_8X8;
      break;

    case VL53L8CX_RESOLUTION_8X8:
      res = VL53L8CX_RESOLUTION_4X4;
      break;

    default:
      break;
  }
  status = sensor_vl53l8cx_top.set_resolution(res);
  status = sensor_vl53l8cx_top.start_ranging();
}

void toggle_signal_and_ambient(void)
{
  EnableAmbient = (EnableAmbient) ? false : true;
  EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void)
{
  snprintf(report, sizeof(report), "%c[2J", 27); /* 27 is ESC command */
  SerialPort.print(report);
}

void handle_cmd(uint8_t cmd)
{
  switch (cmd) {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}