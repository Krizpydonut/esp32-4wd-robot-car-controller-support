#include "esp_camera.h"

#include <Bluepad32.h> 

// Select camera model
#define CAMERA_MODEL_AI_THINKER

// *** Bluepad32 Global Definitions ***
GamepadPtr myGamepad; 

// *** External Declarations ***
extern int speed; 
extern void robot_stop();
extern void robot_setup();
extern void robot_fwd();
extern void robot_back();
extern void robot_left();
extern void robot_right();


// --- CAMERA PIN DEFINITIONS ---
#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5 
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22


#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

extern int gpLed =  4; // Light

// --- Bluepad32 Callback Functions ---
void onConnectedController(ControllerPtr ctl) {
    Serial.printf("CALLBACK: Controller is connected.\n");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    // Assign the new controller to the single gamepad pointer
    myGamepad = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    // Check if the disconnected controller is the one we are currently using
    if (myGamepad == ctl) {
      Serial.println("CALLBACK: Primary controller disconnected.");
      myGamepad = nullptr; 
      robot_stop(); 
    } else {
      Serial.println("CALLBACK: Secondary controller disconnected (ignored).");
    }
}

// --- Robot Control Logic Function ---

void processRobotControl(ControllerPtr ctl) {
    // We already check for isConnected and isGamepad in loop(), but good safety check
    if (!ctl->isGamepad() || !ctl->hasData()) {
        robot_stop();
        return;
    }
    
    // --- Motor Control (using D-pad masks) ---
    uint8_t dpad = ctl->dpad(); 
    
    if (dpad & DPAD_UP) { 
      robot_fwd(); 
    } 
    else if (dpad & DPAD_DOWN) { 
      robot_back(); 
    } 
    else if (dpad & DPAD_LEFT) { 
      robot_left(); 
    } 
    else if (dpad & DPAD_RIGHT) { 
      robot_right(); 
    } 
    else {
      // Stop if no directional button is pressed
      robot_stop(); 
    }

    // --- LED Control (using button masks) ---
    uint32_t buttons = ctl->buttons(); 
    
    // Y Button (Light ON)
    if (buttons & BUTTON_Y) { 
      digitalWrite(gpLed, HIGH);
    }
    
    // A Button (Light OFF)
    if (buttons & BUTTON_A) { 
      digitalWrite(gpLed, LOW);
    }

    if (buttons & BUTTON_X) {
      robot_stop();
    }
}


void setup() {
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();
  robot_setup();
  pinMode(gpLed, OUTPUT);
  digitalWrite(gpLed, LOW);

  // --- Bluepad32 Initialization ---
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Bluepad32 initialized. Waiting for controller...");
  
  BP32.forgetBluetoothKeys(); 

  // --- Camera Configuration ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); 
  s->set_hmirror(s, 0); 
  s->set_vflip(s, 1);    

  // *** Wi-Fi AP and Camera Server start REMOVED ***
  Serial.println("Camera Ready!");
  digitalWrite(33,LOW);
}

void loop() 
{
  // This call fetches all the controllers' data.
  bool dataUpdated = BP32.update();
  
  if (dataUpdated) {
      if (myGamepad && myGamepad->isConnected()) {
          processRobotControl(myGamepad);
      } else {
          robot_stop();
      }
  }

  delay(20);
}