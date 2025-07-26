#include "config.h"      // Incluir archivo de configuración personalizado
#include "cert.h"        // Incluir archivo de certificados de seguridad SSL
#include "esp_camera.h"  // Incluir biblioteca para el manejo de la cámara en ESP
#include <Arduino.h>     // Incluir la biblioteca base de Arduino
#include <ArduinoJson.h> // Incluir la biblioteca de manejo de datos tipo JSON
#include <PCF8574.h>     // Incluir la biblioteca de manejo del expansor de GPIO
#include <Preferences.h> // Incluir la biblioteca que permite almacenar datos en la flash (para guardar el estado de los pines de salida)

//VARIABLES GLOBALES
HardwareSerial SerialAT(1);  // Crear una instancia de HardwareSerial en el puerto 1 para comunicación AT

bool retryFlag = false;          // Bandera que indica si se debe reintentar el último comando AT
bool cameraInitialized = false;  // Bandera para verificar si la cámara ha sido inicializada

Preferences preferences;         // Instancia para el almacenamiento de datos en la flash

//FUNCIONES

// Función que parpadea el LED del módulo x veces con periodo t (milisegundos)
void blinkLed(char xTimes, char tPeriod) {
  for(char i = 0; i < xTimes; i++) {
    digitalWrite(PCIE_LED_PIN, HIGH);
    delay(tPeriod);
    digitalWrite(PCIE_LED_PIN, LOW);
    delay(tPeriod);
  }
}

// Función para recibir la respuesta de un comando AT o de un paquete de datos enviado por serial al módulo A7670SA
String receiveResponseFromModule(int timeout) {
  String response = "";                 // Inicializar cadena vacía para almacenar la respuesta
  long start = millis();                // Guardar el tiempo actual para controlar el timeout

  // Mientras no se supere el tiempo de espera especificado o no se reciban datos
    while ((millis() - start) < timeout) {   
    //Si llegaron datos
    if(SerialAT.available()) {
      while (SerialAT.available()) {      // Mientras haya datos disponibles en el buffer de SerialAT
        char c = SerialAT.read();         // Leer un carácter del buffer
        response += c;                    // Acumular el carácter en la cadena de respuesta      
        
        #if USE_DEBUG_MSG == 1
          // Imprimir cada carácter recibido en tiempo real en el monitor serial
          Serial.print(c);
        #endif
      }
      timeout = millis() - start + 500;   //Esperar 500 milisegundos más por si faltan datos
    }
  }

  return response;
}

// Función para enviar un comando AT y obtener la respuesta del módulo
String sendATCommand(String command, int timeout = 3000) {
  #if USE_DEBUG_MSG == 1
    Serial.print("Enviando comando: ");   // Mostrar en el monitor serial que se enviará un comando
    Serial.print(command);                // Imprimir el comando que se enviará
  #endif
  
  SerialAT.print(command + "\r\n");     // Enviar el comando AT, con retorno de carro y nueva línea a través de SerialAT

  String response = receiveResponseFromModule(timeout);

  #if USE_DEBUG_MSG == 1
    //Si no llega una respuesta, imprimir un salto de línea para visualizar mejor el siguiente mensaje
    if(response == "") {
      Serial.println();
    }
  #endif

  return response;                      // Retornar la respuesta obtenida
}

//Función para enviar datos al módulo, adicionales al comando AT, por ejemplo payloads de comandos AT especiales
String sendDataToModule(const char *buffer, int timeout = 3000) {
  #if USE_DEBUG_MSG == 1  
    Serial.println("Enviando datos:");   // Mostrar en el monitor serial que se enviarán datos
    Serial.println(buffer);              // Imprimir el buffer de datos que se enviará
  #endif

  SerialAT.print(buffer);             // Enviar el buffer de datos a través de SerialAT

  String response = receiveResponseFromModule(timeout);
  
  #if USE_DEBUG_MSG == 1
    //Si no llega una respuesta, imprimir un salto de línea para visualizar mejor el siguiente mensaje
    if(response == "") {
      Serial.println();
    }
  #endif

  return response;                      // Retornar la respuesta obtenida
}

// Función para verificar si la respuesta contiene la cadena esperada
bool verifyResponse(String response, const String& expected) {
  if (response.indexOf(expected) != -1) {  // Si la respuesta contiene la cadena esperada
    return true;                           // Retorna verdadero
  } else {
    retryFlag = true;  // De lo contrario, activa la bandera de reintento
    return false;      // Y retorna falso
  }
}

// Función para reiniciar el módulo A7670SA
void restartModule() {
  #if USE_DEBUG_MSG == 1
    Serial.println("Error, reiniciando el sistema...");
  #endif
  sendATCommand("AT+CRESET");    // Enviar comando AT para reiniciar el módulo A7670SA
  blinkLed(4,500);               // Parpadear LED 4 veces lento para indicar que hubo error y se está reiniciando el módulo
  delay(30000);                  // Esperar 30 segundos para dar tiempo al reinicio del módulo
}

// Función para inicializar el módulo A7670SA mediante comandos AT
void initPCIEModule() {
  bool initialized = false;  // Bandera para indicar si el módulo se ha inicializado correctamente

  while (!initialized) {                               // Bucle hasta que la inicialización sea exitosa
    if (!verifyResponse(sendATCommand("AT"), "OK")) {  // Enviar comando "AT" y verificar que la respuesta contenga "OK"
      restartModule();                                 // Si falla, reiniciar el módulo
      continue;                                        // Reiniciar el bucle para volver a intentar
    }
    if (!verifyResponse(sendATCommand("ATE0"), "OK")) {  // Enviar comando "ATE0" para desactivar el eco y verificar la respuesta "OK"
      restartModule();                                   // Si falla, reiniciar el módulo
      continue;                                          // Reiniciar el bucle
    }
    // Configurar APN usando el comando AT+CGDCONT
    char apnCommand[100];                                    // Declarar un arreglo de caracteres para el comando APN
    sprintf(apnCommand, "AT+CGDCONT=1,\"IP\",\"%s\"", APN);  // Formatear el comando usando la variable APN
    if (!verifyResponse(sendATCommand(apnCommand), "OK")) {  // Enviar el comando APN y verificar la respuesta "OK"
      restartModule();                                       // Si falla, reiniciar el módulo
      continue;                                              // Reiniciar el bucle
    }
    // Configurar la autenticación PDP, si es requerida
    char authCommand[100];                                                        // Declarar un arreglo para el comando de autenticación
    sprintf(authCommand, "AT+CGAUTH=1,1,\"%s\",\"%s\"", APN_USER, APN_PASSWORD);  // Formatear el comando con usuario y contraseña
    if (!verifyResponse(sendATCommand(authCommand), "OK")) {                      // Enviar el comando de autenticación y verificar la respuesta "OK"
      restartModule();                                                            // Si falla, reiniciar el módulo
      continue;                                                                   // Reiniciar el bucle
    }
    // Activar el contexto PDP
    if (!verifyResponse(sendATCommand("AT+CGDATA=\"\",1"), "CONNECT")) {  // Enviar comando para activar PDP y esperar "CONNECT"
      restartModule();                                                    // Si falla, reiniciar el módulo
      continue;                                                           // Reiniciar el bucle
    }
    // Verificar que se ha asignado una dirección IP válida
    if (!verifyResponse(sendATCommand("AT+CGPADDR=1"), ".")) {  // Enviar comando para obtener la dirección IP y verificar que contenga un punto
      restartModule();                                          // Si falla, reiniciar el módulo
      continue;                                                 // Reiniciar el bucle
    }
    initialized = true;  // Si todos los pasos son exitosos, marcar el módulo como inicializado
  }
}

// Función para inicializar el servicio MQTT en el módulo
void initMQTT() {
  //Cargar certificados SSL en el módulo (Se encuentran declarados en el archivo cert.h)
  
  #if USE_DEBUG_MSG == 1
    Serial.println("Enviando clientcert.pem");
  #endif

  sendATCommand("AT+CCERTDOWN=\"clientcert.pem\",1220", 2000);  //El primer número es el tamaño en bytes del certificado, el segundo número es el tiempo de espera
  sendDataToModule(clientCert, 2000);
  
  #if USE_DEBUG_MSG == 1
    Serial.println("Enviando clientkey.pem");
  #endif
  sendATCommand("AT+CCERTDOWN=\"clientkey.pem\",1675", 2000);   //El primer número es el tamaño en bytes del certificado, el segundo número es el tiempo de espera
  sendDataToModule(clientKey, 2000);
  
  #if USE_DEBUG_MSG == 1
    Serial.println("Enviando cacert.pem");
  #endif

  sendATCommand("AT+CCERTDOWN=\"cacert.pem\",1188", 2000);      //El primer número es el tamaño en bytes del certificado, el segundo número es el tiempo de espera
  sendDataToModule(rootCACert, 2000);
  
  delay(500);   //Espera corta antes de verificar certificados

  //Verificar que queden cargados los certificados
  if (!verifyResponse(sendATCommand("AT+CCERTLIST"), "OK")) { // Enviar comando para verificar que los certificados hayan quedado cargados
    #if USE_DEBUG_MSG == 1
      //Si falla, avisar por consola
      Serial.println("Error de certificados TLS");
    #endif
  }

  //Configurar el tipo de autenticación (TLS 1.2) y asociar los certificados cargados
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,4", 2000);
  sendATCommand("AT+CSSLCFG=\"authmode\",0,2", 2000);
  sendATCommand("AT+CSSLCFG=\"cacert\",0,\"cacert.pem\"", 2000);
  sendATCommand("AT+CSSLCFG=\"clientcert\",0,\"clientcert.pem\"", 2000);
  sendATCommand("AT+CSSLCFG=\"clientkey\",0,\"clientkey.pem\"", 2000);

  //Iniciar la comunicación MQTT
  sendATCommand("AT+CMQTTSTART", 2000);
  //Crear el cliente MQTT
  String acquireClientMQTT = String("AT+CMQTTACCQ=0,\"") + MQTT_DEVICE_ID + "\",1";
  sendATCommand(acquireClientMQTT, 2000);
  //Asociar la configuración SSL con la conexión MQTT
  sendATCommand("AT+CMQTTSSLCFG=0,0", 2000);

  //Conectarse a AWS
  String connectMQTT = String("AT+CMQTTCONNECT=0,\"") + MQTT_SERVER + ":" + MQTT_PORT + "\",60,1";
  sendATCommand(connectMQTT, 20000);
  
  //Suscribirse al TOPIC correspondiente
  String topicToSubscribe = TOPIC_SUB;
  sendATCommand("AT+CMQTTSUBTOPIC=0," + String(topicToSubscribe.length()) + ",1", 2000); //Parámetros: client index, topic length, QoS
  sendDataToModule(TOPIC_SUB, 2000);
  sendATCommand("AT+CMQTTSUB=0", 2000);
}

//Función para leer los pines de entrada y salida y generar el payload a enviar por MQTT
String buildPayload(PCF8574 *expansorGpio) {  
  //Variables para almacenar los estados de los cuatro pines de salida
  char estadoPines;

  //Leer los estados de los pines de entrada y salida
  estadoPines = expansorGpio->read8();
  
  /*
  estadoPines = 0x05;
  */

  //Construir un payload en formato JSON
  String payload = "{\"input1\": " + String(estadoPines & 0x01) +
                   ",\"input2\": " + String((estadoPines >> 1) & 0x01) +
                   ",\"output1\": " + String((estadoPines >> 2) & 0x01) +
                   ",\"output2\": " + String((estadoPines >> 3) & 0x01) + "}";

  return payload;
}

//Función para publicar un mensaje MQTT con datos del estado de los pines
void publishMQTT(PCF8574 *expansorGpio) {
  //Indicar el TOPIC al cual se va a enviar un mensaje
  String topicToPublish = TOPIC_PUB;
  sendATCommand("AT+CMQTTTOPIC=0," + String(topicToPublish.length()), 2000);  //Parámetros: client index, topic length
  sendDataToModule(TOPIC_PUB, 2000);
  
  //Obtener el Payload
  String payload = buildPayload(expansorGpio);
  
  //Cargar el Payload indicando el tamaño correspondiente
  sendATCommand("AT+CMQTTPAYLOAD=0," + String(payload.length()), 2000); //Parámetros: client index, payload length
  sendDataToModule(payload.c_str(), 2000);

  //Publicar el payload en el TOPIC
  sendATCommand("AT+CMQTTPUB=0,1,60", 2000);

  #if USE_DEBUG_MSG == 1
    Serial.println("Mensaje publicado en el TOPIC: " + String(TOPIC_PUB)); // Informar que el mensaje fue publicado exitosamente
  #endif

  //Parpadear 3 veces rápido, luego de publicar
  blinkLed(3, 100);
}

// Función para revisar si hay mensajes MQTT entrantes y actuar en función del contenido
void checkForIncomingMessages(PCF8574 *expansorGpio, int timeout = 5000) {
  String resp = "";                 //Variable para almacenar la respuesta completa del broker MQTT
  String jsonPayload = "";          //Variable para extraer el payload de la respuesta
  String jsonValue = "";            //Variable para extraer el valor de los datos del payload
  char startIndex, endIndex;        //Variables para marcar los puntos de inicio y fin del payload en la respuesta
  StaticJsonDocument<200> doc;      //Objeto para almacenar el payload en formato JSON (OJO AL TAMAÑO, AJUSTAR SI SE AGREGAN MÁS ELEMENTOS AL PAYLOAD)n 0x27
  char stateOutPin_1, stateOutPin_2;    //Variables para almacenar el estado de los pines

  //Inicializar los estados
  stateOutPin_1 = 0;
  stateOutPin_2 = 0;

  #if USE_DEBUG_MSG == 1
    Serial.println("Revisando mensajes MQTT entrantes del topic: " +  String(TOPIC_SUB) + "...");
  #endif

  resp = receiveResponseFromModule(timeout);
  delay(2000);
  /*Formato de la respuesta
    {
      "output1": "1",
      "output2": "0"
    }

    "1" es encendido y "0" es apagado
  */

  //Evaluar la respuesta
  //Si la respuesta contiene "+CMQTTRXSTART:" significa que se recibió un mensaje desde el servidor MQTT
  if (resp.indexOf("+CMQTTRXSTART:") != -1) {
    //Si la respuesta contiene "output1" significa que es un mensaje de gestión de salidas
    if(resp.indexOf("output1") != -1) {
      //Extraer el payoad del contenido del mensaje
      startIndex = resp.indexOf("{");
      endIndex = resp.indexOf("}");
      jsonPayload = resp.substring(startIndex, endIndex + 1);
      
      #if USE_DEBUG_MSG == 1
        Serial.println("Payload recibido:");
        Serial.println(jsonPayload);
      #endif

      //Convertir el String a JSON
      deserializeJson(doc, jsonPayload);

      //Identificar si hay que activar/desactivar la salida 1
      if(doc["output1"] == "1") {
        //Encender salida 1
        expansorGpio->write(OUT1_PIN, HIGH);
        stateOutPin_1 = 1;
        
        #if USE_DEBUG_MSG == 1
          Serial.println("OUT_1 encendido");
        #endif
        delay(5000);
        expansorGpio->write(OUT1_PIN, LOW);
        stateOutPin_1 = 0;
      }
      else {
        //Apagar salida 1
        expansorGpio->write(OUT1_PIN, LOW);
        stateOutPin_1 = 0;

        #if USE_DEBUG_MSG == 1
          Serial.println("OUT_1 apagado");
        #endif
      }

      //Identificar si hay que activar/desactivar la salida 2
      if(doc["output2"] == "1") {
        //Encender salida 2        
        expansorGpio->write(OUT2_PIN, HIGH);
        stateOutPin_2 = 1;

        #if USE_DEBUG_MSG == 1
          Serial.println("OUT_2 encendido");
        #endif
        delay(5000);
        expansorGpio->write(OUT2_PIN, LOW);
        stateOutPin_2 = 0;
      }
      else {
        //Apagar salida 2
        expansorGpio->write(OUT2_PIN, LOW);
        stateOutPin_2 = 0;

        #if USE_DEBUG_MSG == 1
          Serial.println("OUT_2 apagado");
        #endif
      }

      //Actualizar en flash el estado de los pines
      preferences.begin("psr-4g", READ_WRITE_MODE);
      preferences.putChar("out_1", stateOutPin_1);
      preferences.putChar("out_2", stateOutPin_2);
      preferences.end();
    }
  }

  #if USE_DEBUG_MSG == 1
    //Si no llega una respuesta, imprimir un salto de línea para visualizar mejor el siguiente mensaje
    if(resp == "") {
      Serial.println("No se recibieron mensajes del TOPIC: " +  String(TOPIC_SUB));
    }
  #endif
}

// Función para detener el servicio MQTT y liberar recursos
void stopMQTT() {
  // Desconectar la sesión MQTT con el comando AT+CMQTTDISC
  if (!verifyResponse(sendATCommand("AT+CMQTTDISC=0,60", 5000), "+CMQTTDISC: 0,0")) {
    return;                   // Salir si falla la desconexión
  }
  // Liberar el cliente MQTT con el comando AT+CMQTTREL
  if (!verifyResponse(sendATCommand("AT+CMQTTREL=0"), "OK")) {
    return;                   // Salir si falla la liberación
  }
  // Detener el servicio MQTT con el comando AT+CMQTTSTOP
  if (!verifyResponse(sendATCommand("AT+CMQTTSTOP", 5000), "+CMQTTSTOP: 0")) {
    return;                   // Salir si falla detener el servicio
  }
}

// Función para codificar datos en Base64
String base64EncodeData(const uint8_t* data, size_t length) {
  const char* base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";  // Conjunto de caracteres para la codificación Base64
  String encoded = "";                                                                            // Cadena que almacenará el resultado codificado
  int val = 0, valb = -6;                                                                         // Variables auxiliares para el proceso de codificación
  for (size_t i = 0; i < length; i++) {                                                           // Iterar sobre cada byte del dato
    val = (val << 8) + data[i];                                                                   // Desplazar 8 bits a la izquierda y añadir el byte actual
    valb += 8;                                                                                    // Incrementar el contador de bits acumulados
    while (valb >= 0) {                                                                           // Mientras haya al menos 6 bits acumulados
      encoded += base64_chars[(val >> valb) & 0x3F];                                              // Extraer 6 bits y mapearlos al caracter correspondiente de Base64
      valb -= 6;                                                                                  // Reducir el contador de bits en 6
    }
  }
  if (valb > -6) {                                               // Si quedan bits sin procesar
    encoded += base64_chars[((val << 8) >> (valb + 8)) & 0x3F];  // Procesar y añadir el caracter correspondiente
  }
  while (encoded.length() % 4) {  // Mientras la longitud del resultado no sea múltiplo de 4
    encoded += '=';               // Añadir padding '=' según sea necesario
  }
  return encoded;  // Retornar la cadena codificada en Base64
}

// Función que inicializa la cámara (configuración similar a "camera_test")
esp_err_t initCamera() {

  camera_config_t config;                // Declarar una estructura para la configuración de la cámara
  config.ledc_channel = LEDC_CHANNEL_0;  // Asignar el canal LEDC 0 para el control del LED de la cámara
  config.ledc_timer = LEDC_TIMER_0;      // Asignar el temporizador LEDC 0
  config.pin_d0 = CAM_Y2_PIN;            // Configurar el pin D0 para datos de la cámara
  config.pin_d1 = CAM_Y3_PIN;            // Configurar el pin D1
  config.pin_d2 = CAM_Y4_PIN;            // Configurar el pin D2
  config.pin_d3 = CAM_Y5_PIN;            // Configurar el pin D3
  config.pin_d4 = CAM_Y6_PIN;            // Configurar el pin D4
  config.pin_d5 = CAM_Y7_PIN;            // Configurar el pin D5
  config.pin_d6 = CAM_Y8_PIN;            // Configurar el pin D6
  config.pin_d7 = CAM_Y9_PIN;            // Configurar el pin D7
  config.pin_xclk = CAM_XCLK_PIN;        // Configurar el pin XCLK (señal de reloj para la cámara)
  config.pin_pclk = CAM_PCLK_PIN;        // Configurar el pin PCLK (señal de píxel)
  config.pin_vsync = CAM_VSYNC_PIN;      // Configurar el pin VSYNC (sincronización vertical)
  config.pin_href = CAM_HREF_PIN;        // Configurar el pin HREF (referencia horizontal)
  config.pin_sccb_sda = CAM_SIOD_PIN;    // Configurar el pin SDA para el bus SCCB
  config.pin_sccb_scl = CAM_SIOC_PIN;    // Configurar el pin SCL para el bus SCCB
  config.pin_pwdn = CAM_PWDN_PIN;        // Configurar el pin para poner la cámara en modo de bajo consumo (power down)
  config.pin_reset = CAM_RESET_PIN;      // Configurar el pin de reset de la cámara
  config.xclk_freq_hz = 20000000;        // Establecer la frecuencia del clock XCLK en 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;  // Establecer el formato de píxel en JPEG

  if (psramFound()) {                    // Verificar si se encontró PSRAM en el sistema
    #if USE_DEBUG_MSG == 1
      Serial.println("PSRAM detectada");
    #endif
    config.frame_size = FRAMESIZE_UXGA;  // Configurar el tamaño de imagen a UXGA si PSRAM está presente
    config.jpeg_quality = 10;            // Establecer la calidad JPEG (valor menor indica mayor calidad)
    config.fb_count = 2;                 // Usar dos buffers de frame
  } else {
    #if USE_DEBUG_MSG == 1
      Serial.println("PSRAM no encontrada");
    #endif
    config.frame_size = FRAMESIZE_SVGA;      // Configurar el tamaño de imagen a SVGA si no hay PSRAM
    config.jpeg_quality = 12;                // Establecer la calidad JPEG a 12
    config.fb_count = 1;                     // Usar un único buffer de frame
    config.fb_location = CAMERA_FB_IN_DRAM;  // Ubicar el buffer en la DRAM
  }

  //Configuración de pines de reloj de la cámara
  pinMode(CAM_PCLK_PIN, INPUT_PULLUP);
  pinMode(CAM_XCLK_PIN, INPUT_PULLUP);

  esp_err_t err = esp_camera_init(&config);                        // Inicializar la cámara con la configuración establecida
  if (err != ESP_OK) {                                             // Si ocurre un error en la inicialización
    #if USE_DEBUG_MSG == 1
      Serial.printf("Error al inicializar la cámara: 0x%x\n", err);  // Imprimir el error en el monitor serial
    #endif

    return err;                                                    // Retornar el código de error
  }

  sensor_t* s = esp_camera_sensor_get();    // Obtener el sensor de la cámara
  #if USE_DEBUG_MSG == 1
    Serial.print("Sensor detectado: ");
    Serial.println(s->id.PID);
  #endif
  if (s->id.PID == OV2640_PID) {            // Si el sensor es del modelo OV2640
    #if USE_DEBUG_MSG == 1
      Serial.println("Sensor OV2640 detectado");  
    #endif
    s->set_brightness(s, 0);                // Ajustar el brillo a 0
    s->set_contrast(s, 1);                  // Ajustar el contraste a 1
    s->set_awb_gain(s, 1);                  // Activar la ganancia automática de balance de blancos
    s->set_saturation(s, 2);                // Ajustar la saturación a 2
    s->set_framesize(s, FRAMESIZE_QVGA);    // Establecer el tamaño del frame a QVGA
    s->set_gainceiling(s, GAINCEILING_8X);  // Establecer el límite de ganancia a 8X
    s->set_lenc(s, 1);                      // Activar la corrección de lente
  }

  return ESP_OK;  // Retornar ESP_OK indicando que la inicialización fue exitosa
}

// Función para desinicializar la cámara y liberar recursos
void deinitCamera() {
  #if USE_DEBUG_MSG == 1
    Serial.println("Liberando cámara...");  // Imprimir mensaje de liberación de la cámara
  #endif
  
  esp_camera_deinit();                    // Llamar a la función para desinicializar la cámara

  #if USE_DEBUG_MSG == 1
    Serial.println("Cámara liberada.");     // Confirmar en el monitor serial que la cámara fue liberada
  #endif
}

// Función para capturar una imagen, codificarla en Base64 y enviarla a RabbitMQ
void sendImageToRabbitMQ(PCF8574 *expansorGpio) {
  
  #if USE_DEBUG_MSG == 1
    Serial.println("Tomando foto...");
  #endif

  //Encender LEDS de flash
  expansorGpio->write(FLASH_LED_1, HIGH);
  expansorGpio->write(FLASH_LED_2, HIGH);  

  // Capturar imagen
  delay(1500);                                      // Esperar 1.5 segundos antes de capturar la imagen
  camera_fb_t* fb = esp_camera_fb_get();            // Capturar un frame (imagen) de la cámara
  if (!fb) {                                        // Verificar si la captura falló
    #if USE_DEBUG_MSG == 1
      Serial.println("Error al capturar la imagen");  // Imprimir mensaje de error en el monitor serial
    #endif

    return;                                         // Salir de la función si no se capturó la imagen
  }

  //Apagar LEDS de flash
  expansorGpio->write(FLASH_LED_1, LOW);
  expansorGpio->write(FLASH_LED_2, LOW);

  //Parpadear 1 vez rápido, cuando tome la foto
  blinkLed(1, 100);
  
  // Convertir la imagen capturada a Base64
  String encodedImg = base64EncodeData(fb->buf, fb->len);  // Codificar el buffer de la imagen en Base64
  esp_camera_fb_return(fb);                                // Devolver el buffer a la cámara para liberar la memoria

  // Iniciar la comunicación HTTP mediante comandos AT
  sendATCommand("AT+HTTPTERM");                                     // Terminar cualquier sesión HTTP previa
  if (!verifyResponse(sendATCommand("AT+HTTPINIT", 8000), "OK")) {  // Inicializar HTTP y verificar respuesta "OK" (timeout de 8 segundos)
    return;                                                         // Salir de la función si la inicialización falla
  }

  // Configurar la URL del servidor RabbitMQ
  String urlCommand = String("AT+HTTPPARA=\"URL\",\"http://") + RABBIT_SERVER + ":" + String(RABBIT_PORT) + ENDPOINT + "\"";
  // Construir el comando AT para establecer la URL usando el servidor, puerto y endpoint definidos
  if (!verifyResponse(sendATCommand(urlCommand), "OK")) {  // Enviar el comando y verificar que se reciba "OK"
    return;                                                // Salir si la configuración de la URL falla
  }

  // Configurar la autenticación básica (usuario y contraseña en Base64)
  String credentials = String(RABBIT_USER) + ":" + RABBIT_PASSWORD;  // Combinar el usuario y contraseña en un string
  String authHeader = "Authorization: Basic " + base64EncodeData((const uint8_t*)credentials.c_str(), credentials.length());
  // Codificar las credenciales en Base64 y formar el header de autorización
  String authCommand = String("AT+HTTPPARA=\"USERDATA\",\"") + authHeader + "\"";  // Construir el comando AT para la autenticación
  if (!verifyResponse(sendATCommand(authCommand), "OK")) {                         // Enviar el comando y verificar la respuesta "OK"
    return;                                                                        // Salir si la autenticación falla
  }

  // Configurar el tipo de contenido para la solicitud HTTP
  String contentTypeCommand = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";  // Establecer el contenido como JSON
  if (!verifyResponse(sendATCommand(contentTypeCommand), "OK")) {              // Enviar el comando y verificar respuesta "OK"
    return;                                                                    // Salir si falla la configuración del contenido
  }

  // Construir el payload en formato JSON, incluyendo la imagen codificada en Base64
  String payload = String("{\"routing_key\":\"") + ROUTING_KEY + String("\",\"payload\":\"") + encodedImg + String("\",\"payload_encoding\":\"base64\",\"properties\":{}}");
  // El payload incluye la clave de enrutamiento, la imagen codificada, el método de codificación y propiedades adicionales

  // Indicar el tamaño del payload y esperar el prompt "DOWNLOAD" para enviar datos
  String dataCommand = String("AT+HTTPDATA=") + payload.length() + ",10000";  // Establecer el tamaño de los datos y un timeout de 10 segundos
  if (!verifyResponse(sendATCommand(dataCommand), "DOWNLOAD")) {              // Enviar el comando y verificar que se reciba "DOWNLOAD"
    return;                                                                   // Salir si no se recibe el prompt esperado
  }

  // Enviar el payload con la imagen en formato JSON
  #if USE_DEBUG_MSG == 1
    Serial.println("Enviando Imagen a RabbitMQ...");
  #endif

  sendATCommand(payload);  // Enviar los datos JSON al servidor

  // Ejecutar la solicitud POST (1 indica método POST)
  if (!verifyResponse(sendATCommand("AT+HTTPACTION=1"), "200")) {  // Ejecutar el POST y verificar que la respuesta contenga "200" (éxito HTTP)
    sendATCommand("AT+HTTPREAD");                                  // Si falla, leer la respuesta del servidor para diagnosticar el error
    return;                                                        // Salir de la función
  }

  // Finalizar la sesión HTTP
  sendATCommand("AT+HTTPTERM");  // Terminar la sesión HTTP para liberar recursos

  //Parpadear 2 veces rápido, cuando transmita por HTTP a Rabbit
  blinkLed(2, 100);
}

// Función para verificar si la cámara está inicializada y, si no, inicializarla
void verificarEInicializarCamara() {
  if (!cameraInitialized) {                                  // Si la cámara aún no ha sido inicializada
    #if USE_DEBUG_MSG == 1
      Serial.println("Inicializando la cámara...");            // Imprimir mensaje indicando que se iniciará la cámara
    #endif

    esp_err_t err = initCamera();                            // Intentar inicializar la cámara
    if (err == ESP_OK) {                                     // Si la inicialización fue exitosa
      cameraInitialized = true;                              // Marcar la cámara como inicializada
      
      #if USE_DEBUG_MSG == 1
        Serial.println("¡Cámara inicializada correctamente!");  // Informar éxito en la inicialización
      #endif
    } else {
      #if USE_DEBUG_MSG == 1
        Serial.println("Error al inicializar la cámara. Intentando reinicio...");  // Informar error y que se intentará un reinicio
      #endif

      deinitCamera();                                                            // Desinicializar la cámara para limpiar recursos
      delay(1000);                                                               // Esperar 1 segundo antes de reintentar
      err = initCamera();                                                        // Intentar inicializar la cámara nuevamente
      if (err == ESP_OK) {                                                       // Si la segunda inicialización es exitosa
        cameraInitialized = true;                                                // Marcar la cámara como inicializada
        
        #if USE_DEBUG_MSG == 1
          Serial.println("Cámara inicializada correctamente tras reinicio.");      // Informar éxito tras el reinicio
        #endif
      } else {
        #if USE_DEBUG_MSG == 1
          Serial.println("Error crítico: No se pudo inicializar la cámara.");  // Informar un error crítico si falla nuevamente
        #endif
      }
    }
  } else {
    #if USE_DEBUG_MSG == 1
      Serial.println("Cámara ya estaba inicializada.");  // Si la cámara ya está inicializada, informar al usuario
    #endif
  }
}

// Función para imprimir la razón por la que el dispositivo despertó del Deep Sleep
esp_sleep_wakeup_cause_t printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason;        // Variable para almacenar la causa del despertar
  wakeup_reason = esp_sleep_get_wakeup_cause();  // Obtener la causa del despertar
  switch (wakeup_reason) {                       // Evaluar la causa del despertar
    case ESP_SLEEP_WAKEUP_EXT0:
      #if USE_DEBUG_MSG == 1
        Serial.println("Wakeup caused by external signal using RTC_IO");  // Despertar por señal externa usando RTC_IO
      #endif
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      #if USE_DEBUG_MSG == 1
        Serial.println("Wakeup caused by external signal using RTC_CNTL");  // Despertar por señal externa usando RTC_CNTL
      #endif
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      #if USE_DEBUG_MSG == 1
        Serial.println("Wakeup caused by timer");  // Despertar causado por el temporizador
      #endif
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      #if USE_DEBUG_MSG == 1
        Serial.println("Wakeup caused by touchpad");  // Despertar causado por el touchpad
      #endif
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      #if USE_DEBUG_MSG == 1
        Serial.println("Wakeup caused by ULP program");  // Despertar causado por un programa ULP
      #endif
      break;
    default:
      #if USE_DEBUG_MSG == 1
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);  // Otro tipo de despertar no relacionado con Deep Sleep
      #endif
      break;
  }

  return wakeup_reason;
}

// Función para esperar a que el módulo A7670SA inicie. Imprime los mensajes que genera en el proceso.
void waitModuleInit(int timeout) {
  long start = millis();                  // Guardar el tiempo actual para controlar el timeout
  while ((millis() - start) < timeout) {  // Mientras no se supere el tiempo de espera especificado
    while (SerialAT.available()) {        // Mientras haya datos disponibles en el buffer de SerialAT
      char c = SerialAT.read();           // Leer un carácter del buffer
      // Imprimir cada carácter recibido en tiempo real en el monitor serial
      #if USE_DEBUG_MSG == 1
        Serial.print(c);
      #endif
    }
  }
}

// Función para configurar las salidas según el último estado almacenado
void setPinsFromMemory(PCF8574 *expansorGpio) {
  char stateOutPin_1, stateOutPin_2;    //Variables para almacenar el estado de los pines

  //Apagar los pines correspondientes a los LED de flash
  expansorGpio->write(FLASH_LED_1, LOW);
  expansorGpio->write(FLASH_LED_2, LOW);

  //Iniciar el espacio de datos para verificar si ya hay algo escrito
  preferences.begin("psr-4g", READ_ONLY_MODE);
  if(!preferences.isKey("out_1")) {
    //Si no existe la clave "out_1" es porque no se ha grabado nada en la flash
    #if USE_DEBUG_MSG == 1
      Serial.println("No hay datos de pines almacenados en flash");
    #endif

    preferences.end();                            //Cerrar el espacio
    preferences.begin("psr-4g", READ_WRITE_MODE); //Abrirlo en modo de escritura/lectura
    //Iniciar el estado de los pines en 0
    preferences.putChar("out_1", 0);              
    preferences.putChar("out_2", 0);
    //Actualizar el estado de los pines de salida
    expansorGpio->write(OUT1_PIN, LOW);
    expansorGpio->write(OUT2_PIN, LOW);
    preferences.end();                            //Cerrar el espacio
    
    #if USE_DEBUG_MSG == 1
      Serial.println("Datos creados correctamente y puestos en cero");
    #endif
  }
  else {
    #if USE_DEBUG_MSG == 1
      Serial.println("Ya existen datos de pines en flash");
    #endif
    stateOutPin_1 = preferences.getChar("out_1");   //Leer el estado almacenado para el pin de salida 1
    stateOutPin_2 = preferences.getChar("out_2");   //Leer el estado almacenado para el pin de salida 2
    //Actualizar el estado de los pines de salida
    expansorGpio->write(OUT1_PIN, stateOutPin_1);
    expansorGpio->write(OUT2_PIN, stateOutPin_2);
    preferences.end();                            //Cerrar el espacio

    #if USE_DEBUG_MSG == 1
      Serial.println("Datos leidos correctamente:");
      Serial.print("Salida 1: ");
      Serial.println(int(stateOutPin_1));
      Serial.print("Salida 2: ");
      Serial.println(int(stateOutPin_2));
    #endif
  }
}

// Función de configuración del sistema, en este caso corre el flujo principal también
void setup() {
  #if USE_DEBUG_MSG == 1
    Serial.begin(115200);                      // Inicializar la comunicación serial a 115200 baudios para depuración
    Serial.println("Dispositivo despierto.");  // Informar que el dispositivo está despierto
  #endif

  //Imprimir la razón por la que el dispositivo despertó
  //A futuro esto puede servir para saber si hubo un reinicio inesperado
  printWakeupReason();

  //Inicializar comunicación I2C con el expansor de pines
  Wire.begin(GROVE_SDA, GROVE_SCL, GROVE_IC2_CLK); //Inicializar I2C en los pines del conector Grove

  //Crear una instancia para el manejo de los pines externos (PCF8574) en la dirección I2C 0x27
  PCF8574 expansorGpio(0x27);

  //Inicializar los pines de entrada y salida externos (PCF8574)
  expansorGpio.setButtonMask(0x03);     //Pines 0 y 1 como entradas, el resto como salidas

  //Leer el último estado almacenado de los pines
  setPinsFromMemory(&expansorGpio);

  // Inicializar la comunicación SerialAT para el módulo A7670SA
  SerialAT.begin(115200, SERIAL_8N1, PCIE_RX_PIN, PCIE_TX_PIN);  // Configurar SerialAT con 115200 baudios, 8 bits, sin paridad y 1 bit de parada, asignando pines RX y TX
  // Inicializar pines
  gpio_deep_sleep_hold_dis();                                      // Deshabilitar la retención de pines durante Deep Sleep
  gpio_hold_dis((gpio_num_t)PCIE_LED_PIN);                        // Liberar estado del pin del LED indicador
  pinMode(PCIE_PWR_PIN, OUTPUT);                                 // Configurar el pin de poder del módulo como salida
  digitalWrite(PCIE_PWR_PIN, LOW);
  pinMode(PCIE_LED_PIN, OUTPUT);                                 // Configurar el pin del LED del módulo como salida
  digitalWrite(PCIE_LED_PIN, LOW);                               // Apagar el LED

  // Procedimiento para encender correctamente el módulo A7670SA:
  pinMode(PWR_ON_PIN, OUTPUT);
  digitalWrite(PWR_ON_PIN, HIGH);
  delay(100);
  digitalWrite(PCIE_PWR_PIN, HIGH);   // Poner en alto el pin de poder para iniciar el encendido
  delay(500);                         // Mantener el pin en bajo durante 0.5 segundos
  digitalWrite(PCIE_PWR_PIN, LOW);    // Poner en bajo el pin de poder para encender el módulo
  
  // Parpadear LED 2 veces lento para indicar que el sistema está encendido
  blinkLed(2, 500);

  // Esperar 30 segundos para asegurar que el módulo A7670SA se encienda completamente
  waitModuleInit(30000);

  #if USE_DEBUG_MSG == 1
    Serial.println("Módulo A7670SA encendido.");  // Informar que el módulo ha sido encendido
  #endif

  // Verificar si la conexión PDP está activa enviando "AT+CGDATA" y esperando "CONNECT"
  if (verifyResponse(sendATCommand("AT+CGDATA=\"\",1"), "CONNECT")) {  
    sendATCommand("ATE0");                                             //Desactivar el eco porque se activa al reiniciar el módulo
    #if USE_DEBUG_MSG == 1
      Serial.println("Conexión PDP activa. Verificando cámara...");      // Informar que la conexión PDP está activa y se procederá a la cámara
    #endif

    verificarEInicializarCamara();                                     // Llamar a la función para inicializar la cámara si aún no lo está
    sendImageToRabbitMQ(&expansorGpio);                                // Capturar y enviar la imagen a RabbitMQ
    initMQTT();                                                        // Inicializar la conexión MQTT
    publishMQTT(&expansorGpio);                                        // Publicar un mensaje MQTT con los datos de los pines
    checkForIncomingMessages(&expansorGpio, INCOMING_MSG_TIMEOUT);     // Revisar si hay mensajes MQTT entrantes y actuar según el contenido
    stopMQTT();                                                        // Detener y desconectar el servicio MQTT
  } else {
    #if USE_DEBUG_MSG == 1
      Serial.println("Conexión PDP fallida. Reiniciando módulo...");    // Informar que la conexión PDP falló y se reiniciará el módulo
    #endif

    initPCIEModule();                                                 // Inicializar (o reiniciar) el módulo PCIe
    verificarEInicializarCamara();                                    // Verificar e inicializar la cámara nuevamente
    sendImageToRabbitMQ(&expansorGpio);                               // Capturar y enviar la imagen a RabbitMQ
    initMQTT();                                                       // Inicializar la conexión MQTT
    publishMQTT(&expansorGpio);                                       // Publicar un mensaje MQTT con los datos de los pines
    checkForIncomingMessages(&expansorGpio, INCOMING_MSG_TIMEOUT);    // Revisar si hay mensajes MQTT entrantes y actuar según el contenido
    stopMQTT();                                                       // Detener y desconectar el servicio MQTT
  }

  // Forzar el modo Sleep en el módulo por comando AT
  sendATCommand("AT+CSCLK=2");             // Enviar comando para activar el modo Sleep en el UART del módulo

  // Parpadear LED 5 veces rápido para indicar que se va a pasar a bajo consumo
  blinkLed(5, 100);

  // Apagar el LED indicador
  digitalWrite(PCIE_LED_PIN, LOW);         // Apagar el LED del módulo
  gpio_hold_en((gpio_num_t)PCIE_LED_PIN);  // Mantener el estado del pin del LED indicador durante Deep Sleep

  // Habilitar la retención de pines durante Deep Sleep
  gpio_deep_sleep_hold_en();
  // Configurar el temporizador para el modo Deep Sleep
  esp_sleep_enable_timer_wakeup(TIEMPO_SLEEP * uS_TO_S_FACTOR);                                   // Configurar el temporizador para despertar después de TIEMPO_SLEEP segundos
  
  #if USE_DEBUG_MSG == 1
    Serial.println("Entrando en modo de Deep Sleep por " + String(TIEMPO_SLEEP) + " segundos...");  // Informar el tiempo de Deep Sleep
    Serial.flush();          // Asegurarse de que todos los mensajes se hayan enviado por el serial
  #endif

  //Poner el sistema en bajo consumo
  esp_deep_sleep_start();  // Iniciar el modo Deep Sleep
}

void loop() {
  // Función loop vacía, ya que todo se ejecuta en setup y no se requiere procesamiento continuo
}
