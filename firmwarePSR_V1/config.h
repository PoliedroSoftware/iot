#ifndef __CONFIG_H__
#define __CONFIG_H__

//Activar o desactivar mensajes de consola
#define USE_DEBUG_MSG   0

//Configuración APN
#define APN "internet.comcel.com.co"
#define APN_USER "comcel"
#define APN_PASSWORD "comcel"

//Configuración RabbitMQ
#define RABBIT_SERVER "3.136.23.237"
#define RABBIT_PORT 15672
#define ENDPOINT "/api/exchanges/%2F/amq.default/publish"
#define RABBIT_USER "guest"
#define RABBIT_PASSWORD "guest"
#define ROUTING_KEY "images-cidei"

//Configuración para la conexión MQTT con el servidor AWS IoT Core
#define MQTT_SERVER           "tcp://a2fzkzjefy7bda-ats.iot.us-east-2.amazonaws.com"
#define MQTT_PORT             8883  // AWS IoT Core usa el puerto 8883 para MQTT seguro
#define MQTT_DEVICE_ID        "psr-4g-at"
#define TOPIC_PUB             "psr/dev/psr-4g-at/data/"
#define TOPIC_SUB             "psr/dev/psr-4g-at/cmd/"
#define MQTT_CLIENT_INDEX     0
#define INCOMING_MSG_TIMEOUT  20000

//Configuración de pines de entrada y salida (Cada número corresponde con el número de pin en el módulo PCF8574)
#define IN1_PIN   0
#define IN2_PIN   1
#define OUT1_PIN  2
#define OUT2_PIN  3

//Configuración de espacio de almacenamiento en flash
#define READ_ONLY_MODE  true
#define READ_WRITE_MODE false

//Configuración de pines para los LED de "flash" (Cada número corresponde con el número de pin en el módulo PCF8574)
#define FLASH_LED_1   5
#define FLASH_LED_2   6

//Configuración de los pines I2C del conector Grove (conector para el módulo PCF8574) y el reloj I2C
#define GROVE_SDA       43      //U0TXD
#define GROVE_SCL       44      //U0RXD
#define GROVE_IC2_CLK   100000  //100 kHz de velocidad I2C

//Configuración Sleep Mode
#define uS_TO_S_FACTOR  1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIEMPO_SLEEP    3600         //segundos

// Versión de la tarjeta T-SIMCAM
// #define USE_SIM_CAM_V1_2
#define USE_SIM_CAM_V1_3    //Add IR Filter

// Definición de pines de la cámara
#define SD_MISO_PIN 40
#define SD_MOSI_PIN 38
#define SD_SCLK_PIN 39
#define SD_CS_PIN 47
#define PCIE_PWR_PIN 48
#define PCIE_TX_PIN 45
#define PCIE_RX_PIN 46
#define PCIE_LED_PIN 21
#define MIC_IIS_WS_PIN 42
#define MIC_IIS_SCK_PIN 41
#define MIC_IIS_DATA_PIN 2
#define CAM_PWDN_PIN -1
#define CAM_RESET_PIN -1
#define CAM_XCLK_PIN 14
#define CAM_SIOD_PIN 4
#define CAM_SIOC_PIN 5
#define CAM_Y9_PIN 15
#define CAM_Y8_PIN 16
#define CAM_Y7_PIN 17
#define CAM_Y6_PIN 12
#define CAM_Y5_PIN 10
#define CAM_Y4_PIN 8
#define CAM_Y3_PIN 9
#define CAM_Y2_PIN 11
#define CAM_VSYNC_PIN 6
#define CAM_HREF_PIN 7
#define CAM_PCLK_PIN 13
#define BUTTON_PIN 0
#define PWR_ON_PIN 1
#define SERIAL_RX_PIN 44
#define SERIAL_TX_PIN 43
#define BAT_VOLT_PIN -1

#if defined(USE_SIM_CAM_V1_3)
#define CAM_IR_PIN 18
#define CAM_RESET_PIN -1
#warning "Currently using V1.3 Pinmap"
#else
#define CAM_RESET_PIN 18
#warning "Currently using V1.2 Pinmap"
#endif

#endif