#include "Watchy.h"

#include <HTTPClient.h>
#include <ArduinoJson.h> 
#include "task_data_json.h" // sadrži json_task_data[]


#ifdef ARDUINO_ESP32S3_DEV
  Watchy32KRTC Watchy::RTC;
  #define ACTIVE_LOW 0
#else
  WatchyRTC Watchy::RTC;
  #define ACTIVE_LOW 1
#endif
GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> Watchy::display(
    WatchyDisplay{});

RTC_DATA_ATTR int guiState;
RTC_DATA_ATTR int menuIndex;
RTC_DATA_ATTR BMA423 sensor;
RTC_DATA_ATTR bool WIFI_CONFIGURED;
RTC_DATA_ATTR bool BLE_CONFIGURED;
RTC_DATA_ATTR weatherData currentWeather;
RTC_DATA_ATTR int weatherIntervalCounter = -1;
RTC_DATA_ATTR long gmtOffset = 0;
RTC_DATA_ATTR bool alreadyInMenu         = true;
RTC_DATA_ATTR bool USB_PLUGGED_IN = false;
RTC_DATA_ATTR tmElements_t bootTime;
RTC_DATA_ATTR uint32_t lastIPAddress;
RTC_DATA_ATTR char lastSSID[30];
RTC_DATA_ATTR watchyAlarm myAlarm = {0, 0, 0, 0, 0, false};  // Initial values of Alarm

#define MAX_COMPONENTS 10
#define MAX_TASKS 10

RTC_DATA_ATTR char componentNames[MAX_COMPONENTS][20];
RTC_DATA_ATTR char taskNames[MAX_TASKS][20];
RTC_DATA_ATTR int taskValues[MAX_COMPONENTS][MAX_TASKS] = {0};
RTC_DATA_ATTR int numComponents = 0;
RTC_DATA_ATTR int numTasks = 0;
RTC_DATA_ATTR bool hasCachedData = false; // da znamo da li uopšte postoji stara tabela

RTC_DATA_ATTR int cursorRow = 0;
RTC_DATA_ATTR int cursorCol = 0;
RTC_DATA_ATTR int viewRow0 = 0;
RTC_DATA_ATTR int viewCol0 = 0;
RTC_DATA_ATTR NavMode navMode = NavMode::ROW;

RTC_DATA_ATTR uint8_t partialCount = 0;      //morao sam da ubacim zbog namestanja ekrana

RTC_DATA_ATTR bool hiddenCol[MAX_TASKS] = {false};
RTC_DATA_ATTR bool hiddenRow[MAX_COMPONENTS] = {false};


void Watchy::init(String datetime) {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause(); // get wake up reason
  #ifdef ARDUINO_ESP32S3_DEV
    Wire.begin(WATCHY_V3_SDA, WATCHY_V3_SCL);     // init i2c
  #else
    Wire.begin(SDA, SCL);                         // init i2c
  #endif
  RTC.init();
  // Init the display since is almost sure we will use it
  display.epd2.initWatchy();

  switch (wakeup_reason) {
  #ifdef ARDUINO_ESP32S3_DEV
  case ESP_SLEEP_WAKEUP_TIMER: // RTC Alarm
  #else
  case ESP_SLEEP_WAKEUP_EXT0: // RTC Alarm
  #endif
    RTC.read(currentTime);
    // Check alarm
    if (myAlarm.active) {
      int8_t currentYear = tmYearToY2k(currentTime.Year);
      if (currentTime.Hour == myAlarm.hour && 
        currentTime.Minute == myAlarm.minute && 
        currentTime.Day == myAlarm.day && 
        currentTime.Month == myAlarm.month &&
        currentYear == myAlarm.year) {
          // Start alarm
          showAlarm();

          myAlarm.active = false;
      }
    }
    switch (guiState) {
    case WATCHFACE_STATE:
      showWatchFace(true); // partial updates on tick
      if (settings.vibrateOClock) {
        if (currentTime.Minute == 0) {
          // The RTC wakes us up once per minute
          vibMotor(75, 4);
        }
      }
      break;
    case MAIN_MENU_STATE:
      // Return to watchface if in menu for more than one tick
      if (alreadyInMenu) {
        guiState = WATCHFACE_STATE;
        showWatchFace(false);
      } else {
        alreadyInMenu = true;
      }
      break;
    }
    break;
  case ESP_SLEEP_WAKEUP_EXT1: // button Press
    handleButtonPress();
    break;
  #ifdef ARDUINO_ESP32S3_DEV
  case ESP_SLEEP_WAKEUP_EXT0: // USB plug in
    pinMode(USB_DET_PIN, INPUT);
    USB_PLUGGED_IN = (digitalRead(USB_DET_PIN) == 1);
    if(guiState == WATCHFACE_STATE){
      RTC.read(currentTime);
      showWatchFace(true);
    }
    break;
  #endif
  default: // reset
    RTC.config(datetime);
    _bmaConfig();
    #ifdef ARDUINO_ESP32S3_DEV
    pinMode(USB_DET_PIN, INPUT);
    USB_PLUGGED_IN = (digitalRead(USB_DET_PIN) == 1);
    #endif    
    gmtOffset = settings.gmtOffset;
    RTC.read(currentTime);
    RTC.read(bootTime);
    showWatchFace(false); // full update on reset
    vibMotor(75, 4);
    // For some reason, seems to be enabled on first boot
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    break;
  }
  deepSleep();
}

void Watchy::deepSleep() {
  display.hibernate();
  RTC.clearAlarm();        // resets the alarm flag in the RTC
  #ifdef ARDUINO_ESP32S3_DEV
  esp_sleep_enable_ext0_wakeup((gpio_num_t)USB_DET_PIN, USB_PLUGGED_IN ? LOW : HIGH); //// enable deep sleep wake on USB plug in/out
  rtc_gpio_set_direction((gpio_num_t)USB_DET_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)USB_DET_PIN);

  esp_sleep_enable_ext1_wakeup(
      BTN_PIN_MASK,
      ESP_EXT1_WAKEUP_ANY_LOW); // enable deep sleep wake on button press
  rtc_gpio_set_direction((gpio_num_t)UP_BTN_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)UP_BTN_PIN);

  rtc_clk_32k_enable(true);
  //rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  int secToNextMin = 60 - timeinfo.tm_sec;
  esp_sleep_enable_timer_wakeup(secToNextMin * uS_TO_S_FACTOR);
  #else
  // Set GPIOs 0-39 to input to avoid power leaking out
  const uint64_t ignore = 0b11110001000000110000100111000010; // Ignore some GPIOs due to resets
  for (int i = 0; i < GPIO_NUM_MAX; i++) {
    if ((ignore >> i) & 0b1)
      continue;
    pinMode(i, INPUT);
  }
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RTC_INT_PIN,
                               0); // enable deep sleep wake on RTC interrupt
  esp_sleep_enable_ext1_wakeup(
      BTN_PIN_MASK,
      ESP_EXT1_WAKEUP_ANY_HIGH); // enable deep sleep wake on button press
  #endif
  esp_deep_sleep_start();
}

void Watchy::handleButtonPress() {
  uint64_t wakeupBit = esp_sleep_get_ext1_wakeup_status();
  // Menu Button
  if (wakeupBit & MENU_BTN_MASK) {
    if (guiState ==
        WATCHFACE_STATE) { // enter menu state if coming from watch face
      showMenu(menuIndex, false);
    } else if (guiState ==
               MAIN_MENU_STATE) { // if already in menu, then select menu item
      switch (menuIndex) {
      case 0:
        showAbout();
        break;
      case 1:
        showBuzz();
        break;
      case 2:
        showAccelerometer();
        break;
      case 3:
        setTime();
        break;
      case 4:
        setupWifi();
        break;
      case 5:
        showUpdateFW();
        break;
      case 6:
        showSyncNTP();
        break;
      case 7:
        setAlarm();
      case 8:
        taskTimes();
      default:
        break;
      }
    } else if (guiState == FW_UPDATE_STATE) {
      updateFWBegin();
    }
  }
  // Back Button
  else if (wakeupBit & BACK_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // exit to watch face if already in menu
      RTC.read(currentTime);
      showWatchFace(false);
    } else if (guiState == APP_STATE) {
      showMenu(menuIndex, false); // exit to menu if already in app
    } else if (guiState == FW_UPDATE_STATE) {
      showMenu(menuIndex, false); // exit to menu if already in app
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }
  // Up Button
  else if (wakeupBit & UP_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // increment menu index
      menuIndex--;
      if (menuIndex < 0) {
        menuIndex = MENU_LENGTH - 1;
      }
      showMenu(menuIndex, true);
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }
  // Down Button
  else if (wakeupBit & DOWN_BTN_MASK) {
    if (guiState == MAIN_MENU_STATE) { // decrement menu index
      menuIndex++;
      if (menuIndex > MENU_LENGTH - 1) {
        menuIndex = 0;
      }
      showMenu(menuIndex, true);
    } else if (guiState == WATCHFACE_STATE) {
      return;
    }
  }

  /***************** fast menu *****************/
  bool timeout     = false;
  long lastTimeout = millis();
  pinMode(MENU_BTN_PIN, INPUT);
  pinMode(BACK_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN, INPUT);
  pinMode(DOWN_BTN_PIN, INPUT);
  while (!timeout) {
    if (millis() - lastTimeout > 5000) {
      timeout = true;
    } else {
      if (digitalRead(MENU_BTN_PIN) == ACTIVE_LOW) {
        lastTimeout = millis();
        if (guiState ==
            MAIN_MENU_STATE) { // if already in menu, then select menu item
          switch (menuIndex) {
          case 0:
            showAbout();
            break;
          case 1:
            showBuzz();
            break;
          case 2:
            showAccelerometer();
            break;
          case 3:
            setTime();
            break;
          case 4:
            setupWifi();
            break;
          case 5:
            showUpdateFW();
            break;
          case 6:
            showSyncNTP();
            break;
          case 7:
            setAlarm();
          case 8:
            taskTimes();
            break;
          default:
            break;
          }
        } else if (guiState == FW_UPDATE_STATE) {
          updateFWBegin();
        }
      } else if (digitalRead(BACK_BTN_PIN) == ACTIVE_LOW) {
        lastTimeout = millis();
        if (guiState ==
            MAIN_MENU_STATE) { // exit to watch face if already in menu
          RTC.read(currentTime);
          showWatchFace(false);
          break; // leave loop
        } else if (guiState == APP_STATE) {
          showMenu(menuIndex, false); // exit to menu if already in app
        } else if (guiState == FW_UPDATE_STATE) {
          showMenu(menuIndex, false); // exit to menu if already in app
        }
      } else if (digitalRead(UP_BTN_PIN) == ACTIVE_LOW) {
        lastTimeout = millis();
        if (guiState == MAIN_MENU_STATE) { // increment menu index
          menuIndex--;
          if (menuIndex < 0) {
            menuIndex = MENU_LENGTH - 1;
          }
          showFastMenu(menuIndex);
        }
      } else if (digitalRead(DOWN_BTN_PIN) == ACTIVE_LOW) {
        lastTimeout = millis();
        if (guiState == MAIN_MENU_STATE) { // decrement menu index
          menuIndex++;
          if (menuIndex > MENU_LENGTH - 1) {
            menuIndex = 0;
          }
          showFastMenu(menuIndex);
        }
      }
    }
  }
}

void Watchy::showMenu(byte menuIndex, bool partialRefresh) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);

  int16_t x1, y1;
  uint16_t w, h;
  int16_t yPos;

  const char *menuItems[] = {
      "About Watchy", "Vibrate Motor", "Show Accelerometer",
      "Set Time",     "Setup WiFi",    "Update Firmware",
      "Sync NTP",     "Set Alarm",    "Task Times"};
  for (int i = 0; i < MENU_LENGTH; i++) {
    yPos = MENU_HEIGHT + (MENU_HEIGHT * i);
    display.setCursor(0, yPos);
    if (i == menuIndex) {
      display.getTextBounds(menuItems[i], 0, yPos, &x1, &y1, &w, &h);
      display.fillRect(x1 - 1, y1 - 10, 200, h + 15, GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.println(menuItems[i]);
    } else {
      display.setTextColor(GxEPD_WHITE);
      display.println(menuItems[i]);
    }
  }

  display.display(partialRefresh);

  guiState = MAIN_MENU_STATE;
  alreadyInMenu = false;
}

void Watchy::showFastMenu(byte menuIndex) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);

  int16_t x1, y1;
  uint16_t w, h;
  int16_t yPos;

  const char *menuItems[] = {
      "About Watchy", "Vibrate Motor", "Show Accelerometer",
      "Set Time",     "Setup WiFi",    "Update Firmware",
      "Sync NTP",     "Set Alarm",    "Task times"};
  for (int i = 0; i < MENU_LENGTH; i++) {
    yPos = MENU_HEIGHT + (MENU_HEIGHT * i);
    display.setCursor(0, yPos);
    if (i == menuIndex) {
      display.getTextBounds(menuItems[i], 0, yPos, &x1, &y1, &w, &h);
      display.fillRect(x1 - 1, y1 - 10, 200, h + 15, GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.println(menuItems[i]);
    } else {
      display.setTextColor(GxEPD_WHITE);
      display.println(menuItems[i]);
    }
  }

  display.display(true);

  guiState = MAIN_MENU_STATE;
}

void Watchy::showAbout() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 20);

  display.print("LibVer: ");
  display.println(WATCHY_LIB_VER);

  display.print("Rev: v");
  display.println(getBoardRevision());

  display.print("Batt: ");
  float voltage = getBatteryVoltage();
  display.print(voltage);
  display.println("V");

  #ifndef ARDUINO_ESP32S3_DEV
  display.print("Uptime: ");
  RTC.read(currentTime);
  time_t b = makeTime(bootTime);
  time_t c = makeTime(currentTime);
  int totalSeconds = c-b;
  //int seconds = (totalSeconds % 60);
  int minutes = (totalSeconds % 3600) / 60;
  int hours = (totalSeconds % 86400) / 3600;
  int days = (totalSeconds % (86400 * 30)) / 86400; 
  display.print(days);
  display.print("d");
  display.print(hours);
  display.print("h");
  display.print(minutes);
  display.println("m");  
  #endif
  
  if(WIFI_CONFIGURED){
    display.print("SSID: ");
    display.println(lastSSID);
    display.print("IP: ");
    display.println(IPAddress(lastIPAddress).toString());
  }else{
    display.println("WiFi Not Connected");
  }
  display.display(false); // full refresh

  guiState = APP_STATE;
}

void Watchy::showBuzz() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(70, 80);
  display.println("Buzz!");
  display.display(false); // full refresh
  vibMotor();
  showMenu(menuIndex, false);
}

void Watchy::vibMotor(uint8_t intervalMs, uint8_t length) {
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  bool motorOn = false;
  for (int i = 0; i < length; i++) {
    motorOn = !motorOn;
    digitalWrite(VIB_MOTOR_PIN, motorOn);
    delay(intervalMs);
  }
}

void Watchy::setTime() {

  guiState = APP_STATE;

  RTC.read(currentTime);

  #ifdef ARDUINO_ESP32S3_DEV
  uint8_t minute = currentTime.Minute;
  uint8_t hour   = currentTime.Hour;
  uint8_t day    = currentTime.Day;
  uint8_t month  = currentTime.Month;
  uint8_t year   = currentTime.Year;  
  #else
  int8_t minute = currentTime.Minute;
  int8_t hour   = currentTime.Hour;
  int8_t day    = currentTime.Day;
  int8_t month  = currentTime.Month;
  int8_t year   = tmYearToY2k(currentTime.Year);
  #endif

  int8_t setIndex = SET_HOUR;

  int8_t blink = 0;

  pinMode(DOWN_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN, INPUT);
  pinMode(MENU_BTN_PIN, INPUT);
  pinMode(BACK_BTN_PIN, INPUT);

  display.setFullWindow();

  while (1) {

    if (digitalRead(MENU_BTN_PIN) == ACTIVE_LOW) {
      setIndex++;
      if (setIndex > SET_DAY) {
        break;
      }
    }
    if (digitalRead(BACK_BTN_PIN) == ACTIVE_LOW) {
      if (setIndex != SET_HOUR) {
        setIndex--;
      }
    }

    blink = 1 - blink;

    if (digitalRead(DOWN_BTN_PIN) == ACTIVE_LOW) {
      blink = 1;
      switch (setIndex) {
      case SET_HOUR:
        hour == 23 ? (hour = 0) : hour++;
        break;
      case SET_MINUTE:
        minute == 59 ? (minute = 0) : minute++;
        break;
      case SET_YEAR:
        year == 99 ? (year = 0) : year++;
        break;
      case SET_MONTH:
        month == 12 ? (month = 1) : month++;
        break;
      case SET_DAY:
        day == 31 ? (day = 1) : day++;
        break;
      default:
        break;
      }
    }

    if (digitalRead(UP_BTN_PIN) == ACTIVE_LOW) {
      blink = 1;
      switch (setIndex) {
      case SET_HOUR:
        hour == 0 ? (hour = 23) : hour--;
        break;
      case SET_MINUTE:
        minute == 0 ? (minute = 59) : minute--;
        break;
      case SET_YEAR:
        year == 0 ? (year = 99) : year--;
        break;
      case SET_MONTH:
        month == 1 ? (month = 12) : month--;
        break;
      case SET_DAY:
        day == 1 ? (day = 31) : day--;
        break;
      default:
        break;
      }
    }

    display.fillScreen(GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&DSEG7_Classic_Bold_53);

    display.setCursor(5, 80);
    if (setIndex == SET_HOUR) { // blink hour digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (hour < 10) {
      display.print("0");
    }
    display.print(hour);

    display.setTextColor(GxEPD_WHITE);
    display.print(":");

    display.setCursor(108, 80);
    if (setIndex == SET_MINUTE) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (minute < 10) {
      display.print("0");
    }
    display.print(minute);

    display.setTextColor(GxEPD_WHITE);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(45, 150);
    if (setIndex == SET_YEAR) { // blink year digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    display.print(2000 + year);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_MONTH) { // blink month digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (month < 10) {
      display.print("0");
    }
    display.print(month);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_DAY) { // blink day digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (day < 10) {
      display.print("0");
    }
    display.print(day);
    display.display(true); // partial refresh
  }

  tmElements_t tm;
  tm.Month  = month;
  tm.Day    = day;
  #ifdef ARDUINO_ESP32S3_DEV
  tm.Year   = year;
  #else
  tm.Year   = y2kYearToTm(year);
  #endif
  tm.Hour   = hour;
  tm.Minute = minute;
  tm.Second = 0;

  RTC.set(tm);

  showMenu(menuIndex, false);
}

void Watchy::setAlarm() {

  guiState = APP_STATE;

  tmElements_t currTime;
  
  RTC.read(currTime);
  
  int8_t minute = currTime.Minute;
  int8_t hour   = currTime.Hour;
  int8_t day    = currTime.Day;
  int8_t month  = currTime.Month;
  int8_t year   = tmYearToY2k(currTime.Year);

  int8_t setIndex = SET_HOUR;

  int8_t blink = 0;

  pinMode(DOWN_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN, INPUT);
  pinMode(BACK_BTN_PIN, INPUT);
  pinMode(MENU_BTN_PIN, INPUT);

  display.setFullWindow();

  while (1) {
    
    if(digitalRead(MENU_BTN_PIN) == ACTIVE_LOW){
      setIndex++;
      if(setIndex > SET_DAY){
        break;
      }
    }
    if(digitalRead(BACK_BTN_PIN) == ACTIVE_LOW){
      if(setIndex != SET_HOUR){
        setIndex--;
      }
    }  

    blink = 1 - blink;

    if(digitalRead(DOWN_BTN_PIN) == ACTIVE_LOW){
      blink = 1;
      switch (setIndex){
        case SET_HOUR:
          hour == 23 ? (hour = 0) : hour++;
          break;
        case SET_MINUTE:
          minute == 59 ? (minute = 0) : minute ++;
          break;
        case SET_YEAR:
          year == 99 ? (year = 0) : year++;
          break;  
        case SET_MONTH:  
          month == 12 ? (month = 1) : month ++;
          break;
        case SET_DAY:
          day == 31 ? (day = 1) : day ++;
          break;
        default:
          break;
      }
    }
    if(digitalRead(UP_BTN_PIN) == ACTIVE_LOW){
      blink = 1;
      switch (setIndex){
        case SET_HOUR:
          hour == 0 ? (hour = 23) : hour--;
          break;
        case SET_MINUTE:
          minute == 0 ? (minute = 59) : minute --;
          break;
        case SET_YEAR:
          year == 0 ? (year = 99) : year--;
          break;  
        case SET_MONTH:  
          month == 1 ? (month = 12) : month --;
          break;
        case SET_DAY:
          day == 1 ? (day = 31) : day --;
          break;
        default:
          break;
      }
    }

    display.fillScreen(GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&DSEG7_Classic_Bold_53);

    display.setCursor(5, 80);
    if(setIndex == SET_HOUR){ // blink hour digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if(hour < 10){
      display.print("0");
    }
    display.print(hour);
    
    display.setTextColor(GxEPD_WHITE);
    display.print(":");

    display.setCursor(108, 80);
    if (setIndex == SET_MINUTE) { // blink minute digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (minute < 10) {
      display.print("0");
    }
    display.print(minute);

    display.setTextColor(GxEPD_WHITE);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(45, 150);
    if (setIndex == SET_YEAR) { // blink year digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    display.print(2000 + year);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_MONTH) { // blink month digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (month < 10) {
      display.print("0");
    }
    display.print(month);

    display.setTextColor(GxEPD_WHITE);
    display.print("/");

    if (setIndex == SET_DAY) { // blink day digits
      display.setTextColor(blink ? GxEPD_WHITE : GxEPD_BLACK);
    }
    if (day < 10) {
      display.print("0");
    }
    display.print(day);
    display.display(true); // partial refresh
  
  }
  
  myAlarm.hour = hour;
  myAlarm.minute = minute;
  myAlarm.day = day;
  myAlarm.month = month;
  myAlarm.year = year;
  myAlarm.active = true;

  showMenu(menuIndex, false);
}

void Watchy::showAlarm() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(70, 80);
  display.println("Alarm!");
  display.display(false); // full refresh
  vibMotor(100, 10);
  switch (guiState) {
    case MAIN_MENU_STATE:
      showMenu(menuIndex, false);
      break;
    default: 
      guiState = WATCHFACE_STATE;
      showWatchFace(false);
      break;   
  }
}

// --- clamp helper ---
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// --- delete funkcije (rade nad TVOJIM taskValues) ---
void Watchy::deleteRow(int row) {
  row = clampi(row, 0, 2);
  for (int c = 0; c < numTasks; ++c) {
    taskValues[row][c] = 0;
  }
}

void Watchy::deleteCol(int col) {
  col = clampi(col, 0, max(0, numTasks - 1));
  for (int r = 0; r < 3; ++r) {
    taskValues[r][col] = 0;
  }
}

void Watchy::fetchTaskData() {
  Serial.println("Pokrenuta fetchTaskData");

  if (hasCachedData) {
    Serial.println("Podaci su već učitani – preskačem fetch");
    return;
  }

  const char* expectedComponents[MAX_COMPONENTS] = {"Hrdwr", "Sftwr", "Frmwr"};
  const char* expectedTasks[MAX_TASKS] = {"Task1", "Task2", "Task3", "Rbt"};

  DynamicJsonDocument doc(2048);
  bool success = false;

  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Povezan na Wi-Fi – pokušavam fetch sa servera...");

    Serial.print("Watchy IP: ");
    Serial.println(WiFi.localIP());

    HTTPClient http;
    http.begin("http://192.168.0.111:5000/data");
    int httpCode = http.GET();

    Serial.print("HTTP status: ");
    Serial.println(httpCode);

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("Primljen JSON:");
      Serial.println(payload);

      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print("Greška pri parsiranju JSON sa Wi-Fi: ");
        Serial.println(error.c_str());
      } else {
        success = true;
        Serial.println("JSON uspešno parsiran sa Wi-Fi servera");
      }
    } else {
      Serial.println("HTTP odgovor nije 200 – problem u vezi/serveru");
    }

    http.end();
    delay(1000);
  }

  // Ako nije uspeo Wi-Fi fetch, pokušaj iz statičkog JSON-a
  if (!success) {
    Serial.println("Koristim statički JSON iz koda (fallback)");

    DeserializationError error = deserializeJson(doc, json_task_data);
    if (error) {
      Serial.print("Greška pri parsiranju statičkog JSON-a: ");
      Serial.println(error.c_str());
      return;
    }

    Serial.println("Učitan statički JSON iz PROGMEM");
  }

  // Učitaj podatke u taskValues iz `doc`

  JsonObject root = doc.as<JsonObject>();

  for (int i = 0; i < 3; i++) {
    strncpy(componentNames[i], expectedComponents[i], sizeof(componentNames[i]));
    JsonObject tasks = root[expectedComponents[i]];

    for (int j = 0; j < 4; j++) {
      if (i == 0) {
        strncpy(taskNames[j], expectedTasks[j], sizeof(taskNames[j]));
      }
      taskValues[i][j] = tasks[expectedTasks[j]];
    }
  }
  numComponents = 3;
  numTasks = 4;
  hasCachedData = true;

  Serial.println("Podaci su učitani i sačuvani u RTC!");
  Serial.print("Broj komponenti: ");
  Serial.println(numComponents);
  Serial.print("Broj taskova: ");
  Serial.println(numTasks);
}

void Watchy::taskTimes() {
  Serial.begin(115200);
  Serial.println("taskTimes pokrenut!");

  for (int i = 0; i < numTasks; ++i) hiddenCol[i] = false;
  for (int i = 0; i < numComponents; ++i) hiddenRow[i] = false;

  // --- (WiFi/JSON tok) ---
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin();
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 10) { delay(500); tries++; }
  }
  hasCachedData = false;
  fetchTaskData(); // puni componentNames/taskNames/taskValues; numComponents=3; numTasks>=1

  if (numComponents == 0 || numTasks == 0) {
    display.setFullWindow();
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 50);
    display.println("Nema podataka!");
    display.display(false);
    return;
  }

  // hard reset drawing state (sprečava "zoom"/isečeni prikaz iz prethodnog partial-a)
  display.setFullWindow();
  display.setTextSize(1);
  display.setRotation(0);
  display.firstPage();
  do { display.fillScreen(GxEPD_WHITE); } while (display.nextPage());


  // --- lokalni helperi (partial/full/indikator) ---

  auto align8 = [](int x) { return x & ~7; };               // poravnaj X na 8 px
  auto span8  = [](int x, int w) {                          // širina -> multipla od 8
    int ax = x & ~7;
    int end = x + w;
    int aend = ((end + 7) & ~7);
    return aend - ax;
  };

  auto buildVisibleCols = [&]() {
    static int vis[ MAX_TASKS ];
    int n = 0;
    for (int i = 0; i < numTasks; ++i) if (!hiddenCol[i]) vis[n++] = i;
    return std::pair<int*,int>(vis, n); // (ptr, count)
  };

  auto buildVisibleRows = [&]() {
    static int vis[ MAX_COMPONENTS ];
    int n = 0;
    for (int i = 0; i < numComponents; ++i) if (!hiddenRow[i]) vis[n++] = i;
    return std::pair<int*,int>(vis, n);
  };

  auto drawFull = [&]() {
    auto [vcols, vcnt] = buildVisibleCols();
    auto [vrows, rcnt] = buildVisibleRows();

    display.setFullWindow();
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);

    // HEADER
    for (int j = 0; j < VCOLS; j++) {
      if (j >= vcnt) break;
      int colIdx = vcols[j];

      int16_t bx, by; uint16_t bw, bh;
      const char *txt = taskNames[colIdx];
      display.getTextBounds(txt, 0, 0, &bx, &by, &bw, &bh);

      int cx = LEFT_X + NAME_COL_W + j*CELL_W;
      int tx = cx + (CELL_W - bw)/2;
      int ty = HEADER_Y;
      display.setCursor(tx, ty);
      display.print(txt);
    }
    display.drawLine(LEFT_X, HEADER_Y + 4, RIGHT_X, HEADER_Y + 4, GxEPD_BLACK);

    // REDOVI popunjavanje svega sa tekstom
    for (int i = 0; i < VROWS; i++) {
      if (i >= rcnt) break;
      int rowIdx = vrows[i];

      int top  = HEADER_Y + 6 + i*ROW_H;
      int base = top + ROW_H - 8;

      display.setCursor(LEFT_X + 2, base);
      display.print(componentNames[rowIdx]);

      for (int j = 0; j < VCOLS; j++) {
        if (j >= vcnt) break;
        int colIdx = vcols[j];

        int x = LEFT_X + NAME_COL_W + j*CELL_W;
        bool focused = (i == cursorRow) && (j == cursorCol);

        char buf[16];
        snprintf(buf, sizeof(buf), "%dms", taskValues[rowIdx][colIdx]);

        int16_t bx, by; uint16_t bw, bh;
        display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        int tx = x + (CELL_W - bw)/2;
        int ty = top + ( (ROW_H - bh) / 2 ) - by;

        display.fillRect(x, top, CELL_W, ROW_H-2, focused ? GxEPD_BLACK : GxEPD_WHITE);
        display.drawRect(x, top, CELL_W, ROW_H-2, GxEPD_BLACK);
        display.setTextColor(focused ? GxEPD_WHITE : GxEPD_BLACK);
        display.setCursor(tx, ty);
        display.print(buf);
      }
      display.drawLine(LEFT_X, top + ROW_H - 2, RIGHT_X, top + ROW_H - 2, GxEPD_BLACK);
    }

    // linije (granica imena + između kolona)
    int gridTop = HEADER_Y - 8, gridBot = HEADER_Y + VROWS*ROW_H + 8;
    display.drawLine(LEFT_X + NAME_COL_W, gridTop, LEFT_X + NAME_COL_W, gridBot, GxEPD_BLACK);
    for (int j = 1; j < VCOLS; j++) {
      int gx = LEFT_X + NAME_COL_W + j*CELL_W;
      display.drawLine(gx, gridTop, gx, gridBot, GxEPD_BLACK);
    }
  
    display.display(false);
  };

  auto drawModeIndicator = [&]() {
    // Široka bela traka da “obriše” istoriju piksela
    const int barH = 40;                 // veća traka = manje ghostinga
    const int y0   = DISPLAY_HEIGHT - barH;    // skroz dole, ali na ekranu
    const int x0   = 0;
    const int w    = DISPLAY_WIDTH;           // 200 (multipla od 8, super)

    display.setPartialWindow(x0, y0, w, barH);
    display.firstPage();
    do {
      display.fillRect(x0, y0, w, barH, GxEPD_WHITE);   // full wipe
      display.drawLine(x0, y0, x0 + w, y0, GxEPD_BLACK);// tanka linija iznad trake

      display.setTextColor(GxEPD_BLACK);
      display.setTextSize(1);
      display.setCursor(x0 + 6, y0 + 14);
      display.print((navMode == NavMode::ROW) ? "[R] Row Mode" : "[C] Col Mode");
    } while (display.nextPage());
  };

  auto redrawCellPartial = [&](int row, int screenCol, bool focused) {
    auto [vcols, vcnt] = buildVisibleCols();
    auto [vrows, rcnt] = buildVisibleRows();
    if (row >= rcnt || screenCol >= vcnt) return;

    int rowIdx = vrows[row];
    int colIdx = vcols[screenCol];

    int top  = HEADER_Y + 6 + row*ROW_H;
    int base = top + ROW_H - 7;
    int x    = LEFT_X + NAME_COL_W + screenCol*CELL_W;
    int w    = CELL_W, h = ROW_H - 2;

    int ax = align8(x);
    int aw = span8(x, w);

    char buf[16];
    snprintf(buf, sizeof(buf), "%dms", taskValues[rowIdx][colIdx]);
    int16_t bx, by; uint16_t bw, bh;
    display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    int tx = x + (CELL_W - bw)/2;
    int ty = top + ( (ROW_H - bh) / 2 ) - by;

    display.setPartialWindow(ax, top, aw, h);
    display.firstPage();
    do {
      display.fillRect(x, top, w, h, focused ? GxEPD_BLACK : GxEPD_WHITE);
      display.drawRect(x, top, w, h, GxEPD_BLACK);
      display.setTextColor(focused ? GxEPD_WHITE : GxEPD_BLACK);
      display.setCursor(tx, ty);
      display.print(buf);
    } while (display.nextPage());
  };

  auto keepCursorVisibleHoriz = [&]() {
    if (cursorCol < viewCol0) viewCol0 = cursorCol;
    if (cursorCol > viewCol0 + VCOLS - 1) viewCol0 = cursorCol - (VCOLS - 1);
    viewCol0 = clampi(viewCol0, 0, max(0, numTasks - VCOLS));
  };

  auto maxStart = [&](){ return max(0, numTasks - VCOLS); };

  auto shiftCols = [&](int dir, bool aggressive){
    int sc = cursorCol - viewCol0;
    int old = cursorCol;

    if (!aggressive) {
      int ns = clampi(viewCol0 + (dir > 0 ? 1 : -1), 0, maxStart());
      if (ns == viewCol0) return;
      viewCol0 = ns;
      cursorCol = clampi(viewCol0 + sc, 0, numTasks - 1);
    } else {
      if (dir > 0) {
        viewCol0 = clampi(old + 1, 0, maxStart());
        cursorCol = clampi(viewCol0, 0, numTasks - 1);
      } else {
        viewCol0 = clampi(old - VCOLS, 0, maxStart());
        cursorCol = clampi(min(old - 1, viewCol0 + VCOLS - 1), 0, numTasks - 1);
      }
    } 

    drawFull();
    drawModeIndicator();
    partialCount = 0;
  };

  auto clampCursorToVisible = [&](){
    auto [vcols, vcnt] = buildVisibleCols();
    auto [vrows, rcnt] = buildVisibleRows();
    if (cursorCol >= vcnt) cursorCol = max(0, vcnt-1);
    if (cursorRow >= rcnt) cursorRow = max(0, rcnt-1);
  };

  // --- init pinova i state ---
  guiState = APP_STATE;
  pinMode(BACK_BTN_PIN, INPUT);
  pinMode(MENU_BTN_PIN, INPUT);
  pinMode(UP_BTN_PIN,   INPUT);
  pinMode(DOWN_BTN_PIN, INPUT);

  cursorRow = clampi(cursorRow, 0, VROWS - 1);
  cursorCol = clampi(cursorCol, 0, VCOLS - 1);
  keepCursorVisibleHoriz();
  drawFull();
  drawModeIndicator();
  partialCount = 0;

  // edge detekcija tastera
  auto rd = [&](int pin){ return digitalRead(pin) == ACTIVE_LOW; };
  bool pUp=rd(UP_BTN_PIN), pDn=rd(DOWN_BTN_PIN), pMn=rd(MENU_BTN_PIN), pBk=rd(BACK_BTN_PIN);
  unsigned long menuHeldAt = 0;
  const unsigned long HELD_MS = 250;

  while (1) {
    bool up = rd(UP_BTN_PIN);
    bool dn = rd(DOWN_BTN_PIN);
    bool mn = rd(MENU_BTN_PIN);
    bool bk = rd(BACK_BTN_PIN);

    // BACK izlaz
    if (bk && !pBk) {
      guiState = MAIN_MENU_STATE;
      display.setFullWindow();
      display.fillScreen(GxEPD_WHITE);
      display.display(false);
      showMenu(menuIndex, false);
      break;
    }

    // MENU hold
    if (mn && !pMn) menuHeldAt = millis();
    bool menuHeld = mn && (millis() - menuHeldAt >= HELD_MS);

    // Kad prvi put uđemo u held, resetuj edge za UP/DOWN
    static bool menuHeldPrev = false;
    if (menuHeld && !menuHeldPrev) {
      // “zaboravi” prethodno stanje tastera da sledeći pritisak bude čist edge
      pUp = false;
      pDn = false;
    }
    menuHeldPrev = menuHeld;

    // MENU tap -> toggle mod
    if (!mn && pMn) {
      bool wasHeld = (millis() - menuHeldAt >= HELD_MS);
      if (!wasHeld) { // kratko
        navMode = (navMode == NavMode::ROW) ? NavMode::COL : NavMode::ROW;
        drawModeIndicator(); // partial
      }
    }

    // Navigacija (bez MENU) -> partial ili full (ako je skrol)
    if (!mn) {
      if (up && !pUp) {
        int oldRow = cursorRow, oldCol = cursorCol;
        if (navMode == NavMode::ROW) cursorRow = clampi(cursorRow - 1, 0, 2);
        else                         cursorCol = clampi(cursorCol - 1, 0, max(0, numTasks - 1));
        bool oldScrolledLeft = (cursorCol < viewCol0);
        keepCursorVisibleHoriz();
        bool scrolled = oldScrolledLeft || (cursorCol > viewCol0 + VCOLS - 1);
        if (scrolled) {
          drawFull();
        } else {
          // partial: stara i nova ćelija
          int oldScreenCol = oldCol - viewCol0;
          if (oldScreenCol >= 0 && oldScreenCol < VCOLS)
            redrawCellPartial(oldRow, oldScreenCol, false);
          int newScreenCol = cursorCol - viewCol0;
          redrawCellPartial(cursorRow, newScreenCol, true);

          // brojanje parcijala
          partialCount += 2;  // dve ćelije parcijalno
          if (partialCount >= 50) {
            drawFull();       // full refresh (drawFull već nacrta i mode traku)
            drawModeIndicator();
            partialCount = 0;
          }
        }
      }
      if (dn && !pDn) {
        int oldRow = cursorRow, oldCol = cursorCol;
        if (navMode == NavMode::ROW) cursorRow = clampi(cursorRow + 1, 0, 2);
        else                         cursorCol = clampi(cursorCol + 1, 0, max(0, numTasks - 1));
        bool oldScrolledRight = (cursorCol > viewCol0 + VCOLS - 1);
        keepCursorVisibleHoriz();
        bool scrolled = (cursorCol < viewCol0) || oldScrolledRight;
        if (scrolled) {
          drawFull();
        } else {
          int oldScreenCol = oldCol - viewCol0;
          if (oldScreenCol >= 0 && oldScreenCol < VCOLS)
            redrawCellPartial(oldRow, oldScreenCol, false);
          int newScreenCol = cursorCol - viewCol0;
          redrawCellPartial(cursorRow, newScreenCol, true);

          partialCount += 2;
          if (partialCount >= 50) {
            drawFull();
            drawModeIndicator();
            partialCount = 0;
          }
        }
      }
    }
    
    if (menuHeld && bk && !pBk) {
      for (int i = 0; i < numTasks; ++i) hiddenCol[i] = false;
      for (int i = 0; i < numComponents; ++i) hiddenRow[i] = false;
      clampCursorToVisible();
      drawFull(); drawModeIndicator(); partialCount = 0;
      continue;
    }

    if (menuHeld) {
      auto [vcols, vcnt] = buildVisibleCols();
      auto [vrows, rcnt] = buildVisibleRows();

      if (navMode == NavMode::COL) {
        // MENU+UP  -> sakrij kolonu levo od fokusa (ako postoji)
        if (up && !pUp && vcnt > 1 && cursorCol > 0) {
          int realLeft = vcols[cursorCol - 1];
          hiddenCol[realLeft] = true;        // “nestane” levo
          clampCursorToVisible();
          drawFull(); drawModeIndicator(); partialCount = 0;
        }
        // MENU+DOWN -> sakrij kolonu desno od fokusa (ako postoji)
        if (dn && !pDn && vcnt > 1 && cursorCol < vcnt - 1) {
          int realRight = vcols[cursorCol + 1];
          hiddenCol[realRight] = true;       // “nestane” desno
          clampCursorToVisible();
          drawFull(); drawModeIndicator(); partialCount = 0;
        }
      } else {
        // ROW mod (gore/dole)
        if (up && !pUp && rcnt > 1 && cursorRow > 0) {
          int realAbove = vrows[cursorRow - 1];
          hiddenRow[realAbove] = true;
          clampCursorToVisible();
          drawFull(); drawModeIndicator(); partialCount = 0;
        }
        if (dn && !pDn && rcnt > 1 && cursorRow < rcnt - 1) {
          int realBelow = vrows[cursorRow + 1];
          hiddenRow[realBelow] = true;
          clampCursorToVisible();
          drawFull(); drawModeIndicator(); partialCount = 0;
        }
      }
    }


    pUp = up; pDn = dn; pMn = mn; pBk = bk;
    delay(60);
  }
}

void Watchy::showAccelerometer() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);

  Accel acc;

  long previousMillis = 0;
  long interval       = 200;

  guiState = APP_STATE;

  pinMode(BACK_BTN_PIN, INPUT);

  while (1) {

    unsigned long currentMillis = millis();

    if (digitalRead(BACK_BTN_PIN) == ACTIVE_LOW) {
      break;
    }

    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      // Get acceleration data
      bool res          = sensor.getAccel(acc);
      uint8_t direction = sensor.getDirection();
      display.fillScreen(GxEPD_BLACK);
      display.setCursor(0, 30);
      if (res == false) {
        display.println("getAccel FAIL");
      } else {
        display.print("  X:");
        display.println(acc.x);
        display.print("  Y:");
        display.println(acc.y);
        display.print("  Z:");
        display.println(acc.z);

        display.setCursor(30, 130);
        switch (direction) {
        case DIRECTION_DISP_DOWN:
          display.println("FACE DOWN");
          break;
        case DIRECTION_DISP_UP:
          display.println("FACE UP");
          break;
        case DIRECTION_BOTTOM_EDGE:
          display.println("BOTTOM EDGE");
          break;
        case DIRECTION_TOP_EDGE:
          display.println("TOP EDGE");
          break;
        case DIRECTION_RIGHT_EDGE:
          display.println("RIGHT EDGE");
          break;
        case DIRECTION_LEFT_EDGE:
          display.println("LEFT EDGE");
          break;
        default:
          display.println("ERROR!!!");
          break;
        }
      }
      display.display(true); // full refresh
    }
  }

  showMenu(menuIndex, false);
}

void Watchy::showWatchFace(bool partialRefresh) {
  display.setFullWindow();
  // At this point it is sure we are going to update
  display.epd2.asyncPowerOn();
  drawWatchFace();
  display.display(partialRefresh); // partial refresh
  guiState = WATCHFACE_STATE;
}

void Watchy::drawWatchFace() {
  display.setFont(&DSEG7_Classic_Bold_53);
  display.setCursor(5, 53 + 60);
  if (currentTime.Hour < 10) {
    display.print("0");
  }
  display.print(currentTime.Hour);
  display.print(":");
  if (currentTime.Minute < 10) {
    display.print("0");
  }
  display.println(currentTime.Minute);
}

/*weatherData Watchy::getWeatherData() {
  return _getWeatherData(settings.cityID, settings.lat, settings.lon,
    settings.weatherUnit, settings.weatherLang, settings.weatherURL,
    settings.weatherAPIKey, settings.weatherUpdateInterval);
}

weatherData Watchy::_getWeatherData(String cityID, String lat, String lon, String units, String lang,
                                   String url, String apiKey,
                                   uint8_t updateInterval) {
  currentWeather.isMetric = units == String("metric");
  if (weatherIntervalCounter < 0) { //-1 on first run, set to updateInterval
    weatherIntervalCounter = updateInterval;
  }
  if (weatherIntervalCounter >=
      updateInterval) { // only update if WEATHER_UPDATE_INTERVAL has elapsed
                        // i.e. 30 minutes
    if (connectWiFi()) {
      HTTPClient http; // Use Weather API for live data if WiFi is connected
      http.setConnectTimeout(3000); // 3 second max timeout
      String weatherQueryURL = url;
      if(cityID != ""){
        weatherQueryURL.replace("{cityID}", cityID);
      }else{
        weatherQueryURL.replace("{lat}", lat);
        weatherQueryURL.replace("{lon}", lon);
      }
      weatherQueryURL.replace("{units}", units);
      weatherQueryURL.replace("{lang}", lang);
      weatherQueryURL.replace("{apiKey}", apiKey);
      http.begin(weatherQueryURL.c_str());
      int httpResponseCode = http.GET();
      if (httpResponseCode == 200) {
        String payload             = http.getString();
        JSONVar responseObject     = JSON.parse(payload);
        currentWeather.temperature = int(responseObject["main"]["temp"]);
        currentWeather.weatherConditionCode =
            int(responseObject["weather"][0]["id"]);
        currentWeather.weatherDescription =
		        JSONVar::stringify(responseObject["weather"][0]["main"]);
	      currentWeather.external = true;
		        breakTime((time_t)(int)responseObject["sys"]["sunrise"], currentWeather.sunrise);
		        breakTime((time_t)(int)responseObject["sys"]["sunset"], currentWeather.sunset);
        // sync NTP during weather API call and use timezone of lat & lon
        gmtOffset = int(responseObject["timezone"]);
        syncNTP(gmtOffset);
      } else {
        // http error
      }
      http.end();
      // turn off radios
      WiFi.mode(WIFI_OFF);
      btStop();
    } else { // No WiFi, use internal temperature sensor
      uint8_t temperature = sensor.readTemperature(); // celsius
      if (!currentWeather.isMetric) {
        temperature = temperature * 9. / 5. + 32.; // fahrenheit
      }
      currentWeather.temperature          = temperature;
      currentWeather.weatherConditionCode = 800;
      currentWeather.external             = false;
    }
    weatherIntervalCounter = 0;
  } else {
    weatherIntervalCounter++;
  }
  return currentWeather;
}*/

float Watchy::getBatteryVoltage() {
  #ifdef ARDUINO_ESP32S3_DEV
    return analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f * ADC_VOLTAGE_DIVIDER;
  #else
  if (RTC.rtcType == DS3231) {
    return analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f *
           2.0f; // Battery voltage goes through a 1/2 divider.
  } else {
    return analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f * 2.0f;
  }
  #endif
}

uint8_t Watchy::getBoardRevision() {
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  if(chip_info.model == CHIP_ESP32){ //Revision 1.0 - 2.0
    Wire.beginTransmission(0x68); //v1.0 has DS3231
    if (Wire.endTransmission() == 0){
      return 10;
    }
    delay(1);
    Wire.beginTransmission(0x51); //v1.5 and v2.0 have PCF8563
    if (Wire.endTransmission() == 0){
        pinMode(35, INPUT);
        if(digitalRead(35) == 0){
          return 20; //in rev 2.0, pin 35 is BTN 3 and has a pulldown
        }else{
          return 15; //in rev 1.5, pin 35 is the battery ADC
        }
    }
  }
  if(chip_info.model == CHIP_ESP32S3){ //Revision 3.0
    return 30;
  }
  return -1;
}

uint16_t Watchy::_readRegister(uint8_t address, uint8_t reg, uint8_t *data,
                               uint16_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (uint8_t)len);
  uint8_t i = 0;
  while (Wire.available()) {
    data[i++] = Wire.read();
  }
  return 0;
}

uint16_t Watchy::_writeRegister(uint8_t address, uint8_t reg, uint8_t *data,
                                uint16_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  return (0 != Wire.endTransmission());
}

void Watchy::_bmaConfig() {

  if (sensor.begin(_readRegister, _writeRegister, delay) == false) {
    // fail to init BMA
    return;
  }

  // Accel parameter structure
  Acfg cfg;
  /*!
      Output data rate in Hz, Optional parameters:
          - BMA4_OUTPUT_DATA_RATE_0_78HZ
          - BMA4_OUTPUT_DATA_RATE_1_56HZ
          - BMA4_OUTPUT_DATA_RATE_3_12HZ
          - BMA4_OUTPUT_DATA_RATE_6_25HZ
          - BMA4_OUTPUT_DATA_RATE_12_5HZ
          - BMA4_OUTPUT_DATA_RATE_25HZ
          - BMA4_OUTPUT_DATA_RATE_50HZ
          - BMA4_OUTPUT_DATA_RATE_100HZ
          - BMA4_OUTPUT_DATA_RATE_200HZ
          - BMA4_OUTPUT_DATA_RATE_400HZ
          - BMA4_OUTPUT_DATA_RATE_800HZ
          - BMA4_OUTPUT_DATA_RATE_1600HZ
  */
  cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
  /*!
      G-range, Optional parameters:
          - BMA4_ACCEL_RANGE_2G
          - BMA4_ACCEL_RANGE_4G
          - BMA4_ACCEL_RANGE_8G
          - BMA4_ACCEL_RANGE_16G
  */
  cfg.range = BMA4_ACCEL_RANGE_2G;
  /*!
      Bandwidth parameter, determines filter configuration, Optional parameters:
          - BMA4_ACCEL_OSR4_AVG1
          - BMA4_ACCEL_OSR2_AVG2
          - BMA4_ACCEL_NORMAL_AVG4
          - BMA4_ACCEL_CIC_AVG8
          - BMA4_ACCEL_RES_AVG16
          - BMA4_ACCEL_RES_AVG32
          - BMA4_ACCEL_RES_AVG64
          - BMA4_ACCEL_RES_AVG128
  */
  cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

  /*! Filter performance mode , Optional parameters:
      - BMA4_CIC_AVG_MODE
      - BMA4_CONTINUOUS_MODE
  */
  cfg.perf_mode = BMA4_CONTINUOUS_MODE;

  // Configure the BMA423 accelerometer
  sensor.setAccelConfig(cfg);

  // Enable BMA423 accelerometer
  // Warning : Need to use feature, you must first enable the accelerometer
  // Warning : Need to use feature, you must first enable the accelerometer
  sensor.enableAccel();

  struct bma4_int_pin_config config;
  config.edge_ctrl = BMA4_LEVEL_TRIGGER;
  config.lvl       = BMA4_ACTIVE_HIGH;
  config.od        = BMA4_PUSH_PULL;
  config.output_en = BMA4_OUTPUT_ENABLE;
  config.input_en  = BMA4_INPUT_DISABLE;
  // The correct trigger interrupt needs to be configured as needed
  sensor.setINTPinConfig(config, BMA4_INTR1_MAP);

  struct bma423_axes_remap remap_data;
  remap_data.x_axis      = 1;
  remap_data.x_axis_sign = 0xFF;
  remap_data.y_axis      = 0;
  remap_data.y_axis_sign = 0xFF;
  remap_data.z_axis      = 2;
  remap_data.z_axis_sign = 0xFF;
  // Need to raise the wrist function, need to set the correct axis
  sensor.setRemapAxes(&remap_data);

  // Enable BMA423 isStepCounter feature
  sensor.enableFeature(BMA423_STEP_CNTR, true);
  // Enable BMA423 isTilt feature
  sensor.enableFeature(BMA423_TILT, true);
  // Enable BMA423 isDoubleClick feature
  sensor.enableFeature(BMA423_WAKEUP, true);

  // Reset steps
  sensor.resetStepCounter();

  // Turn on feature interrupt
  sensor.enableStepCountInterrupt();
  sensor.enableTiltInterrupt();
  // It corresponds to isDoubleClick interrupt
  sensor.enableWakeupInterrupt();
}

void Watchy::setupWifi() {
  /*display.epd2.setBusyCallback(0); // temporarily disable lightsleep on busy
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  wifiManager.setTimeout(WIFI_AP_TIMEOUT);
  wifiManager.setAPCallback(_configModeCallback);
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  if (!wifiManager.autoConnect(WIFI_AP_SSID)) { // WiFi setup failed
    display.println("Setup failed &");
    display.println("timed out!");
  } else {
    display.println("Connected to:");
    display.println(WiFi.SSID());
		display.println("Local IP:");
		display.println(WiFi.localIP());
    weatherIntervalCounter = -1; // Reset to force weather to be read again
    lastIPAddress = WiFi.localIP();
    WiFi.SSID().toCharArray(lastSSID, 30);
  }*/
  display.epd2.setBusyCallback(0); // temporarily disable lightsleep on busy
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
  pinMode(BACK_BTN_PIN, INPUT_PULLUP);

if (WiFi.status() == WL_CONNECTED) {
  display.setCursor(0, 20);
  display.println("Povezan na:");
  display.println(WiFi.SSID());
  display.println("DOWN = diskonektuj");
  display.println("BACK = izadji");
  display.display(false);

  bool prevDown = digitalRead(DOWN_BTN_PIN);
  bool prevBack = digitalRead(BACK_BTN_PIN);

  while (true) {
    bool nowDown = digitalRead(DOWN_BTN_PIN);
    bool nowBack = digitalRead(BACK_BTN_PIN);

    bool downPressed = (prevDown == HIGH && nowDown == LOW); // pritisak
    bool backPressed = (prevBack == HIGH && nowBack == LOW);

    if (downPressed) {
      Serial.println("Wi-Fi diskonektovan rucno.");
      WiFi.disconnect(false, true);
      delay(300);
      WiFi.mode(WIFI_OFF);
      btStop();

      display.fillScreen(GxEPD_BLACK);
      display.setCursor(0, 50);
      display.println("Diskonektovan.");
      display.println("BACK = izadji");
      display.display(false);

      // čekaj dok ne pritisneš BACK
      prevBack = digitalRead(BACK_BTN_PIN);
      while (true) {
        bool nb = digitalRead(BACK_BTN_PIN);
        if (prevBack == HIGH && nb == LOW) { break; }
        prevBack = nb;
        delay(40);
      }
      return;
    }

    if (backPressed) {
      guiState = MAIN_MENU_STATE;
      showMenu(menuIndex, false);  // vrati u meni
      return; // izlaz, ostaje povezan
    }

    prevDown = nowDown;
    prevBack = nowBack;
    delay(40);
  }
}

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(100);

  // NISMO povezani — pokušaj konekcije kao ranije
  WiFiManager wifiManager;
  //wifiManager.resetSettings();

  wifiManager.setTimeout(WIFI_AP_TIMEOUT);
  wifiManager.setAPCallback(_configModeCallback);

  display.setCursor(0, 20);
  display.println("Povezivanje...");
  display.display(false);

  bool ok = wifiManager.autoConnect(WIFI_AP_SSID);
  if (!ok) {                     // autoConnect pao pre nego što si video portal
    wifiManager.setTimeout(180); // 3 min da stigneš da se povežeš
    ok = wifiManager.startConfigPortal(WIFI_AP_SSID);  // eksplicitno otvori AP
  }

  display.fillScreen(GxEPD_BLACK);
  display.setCursor(0, 20);
  if (ok) {
    display.println("Connected to:");
    display.println(WiFi.SSID());
    display.println("IP:");
    display.println(WiFi.localIP());
    display.println("DOWN = diskonektuj");
    display.println("BACK = izadji");
    display.display(false);

    bool prevDown = digitalRead(DOWN_BTN_PIN);
    bool prevBack = digitalRead(BACK_BTN_PIN);

    while (true) {
      bool nowDown = digitalRead(DOWN_BTN_PIN);
      bool nowBack = digitalRead(BACK_BTN_PIN);

      bool downPressed = (prevDown == HIGH && nowDown == LOW);
      bool backPressed = (prevBack == HIGH && nowBack == LOW);

      if (downPressed) {
        Serial.println("Wi-Fi diskonektovan rucno.");
        WiFi.disconnect(false, true);
        delay(300);
        WiFi.mode(WIFI_OFF);
        btStop();

        display.fillScreen(GxEPD_BLACK);
        display.setCursor(0, 50);
        display.println("Diskonektovan.");
        display.println("BACK = izadji");
        display.display(false);

        // čekaj BACK da izađeš
        prevBack = digitalRead(BACK_BTN_PIN);
        while (true) {
          bool nb = digitalRead(BACK_BTN_PIN);
          if (prevBack == HIGH && nb == LOW) { break; }
          prevBack = nb;
          delay(40);
        }
        return;
      }

      if (backPressed) {
        // Ako si samo izašao, upiši meta podatke i ostavi Wi-Fi aktivan
        
        showMenu(menuIndex, false);  // vrati u meni
        while (digitalRead(BACK_BTN_PIN) == LOW) {  // sačekaj da pustiš BACK (ACTIVE_LOW)
          delay(30);
        }
        delay(120); 

        weatherIntervalCounter = -1;
        lastIPAddress = WiFi.localIP();
        WiFi.SSID().toCharArray(lastSSID, 30);
        return;
      }

      prevDown = nowDown;
      prevBack = nowBack;
      delay(40);
    }
  }else {
    display.println("Setup failed");
    display.println("or timed out.");
  }

  display.display(false); // full refresh
  // turn off radios
  //WiFi.mode(WIFI_OFF); // OVO SAM JA ZAKOMENTARISAO DA BI WIFI MOGAO DA RADI SVE VREME DOK GA NE ISKLJUCIM RUCNO NA BACK
  btStop();
  // enable lightsleep on busy
  display.epd2.setBusyCallback(WatchyDisplay::busyCallback);
  guiState = APP_STATE;
}

void Watchy::_configModeCallback(WiFiManager *myWiFiManager) {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Connect to");
  display.print("SSID: ");
  display.println(WIFI_AP_SSID);
  display.print("IP: ");
  display.println(WiFi.softAPIP());
	display.println("MAC address:");
	display.println(WiFi.softAPmacAddress().c_str());
  display.display(false); // full refresh
}

bool Watchy::connectWiFi() {
  if (WL_CONNECT_FAILED ==
      WiFi.begin()) { // WiFi not setup, you can also use hard coded credentials
                      // with WiFi.begin(SSID,PASS);
    WIFI_CONFIGURED = false;
  } else {
    if (WL_CONNECTED ==
        WiFi.waitForConnectResult()) { // attempt to connect for 10s
      lastIPAddress = WiFi.localIP();
      WiFi.SSID().toCharArray(lastSSID, 30);
      WIFI_CONFIGURED = true;
    } else { // connection failed, time out
      WIFI_CONFIGURED = false;
      // turn off radios
      //WiFi.mode(WIFI_OFF);                ZBOG PROJEKTA ISKLJUCENO 
      //btStop();
      Serial.println(" WiFi konekcija nije uspela – ostavljam WiFi uključen za retry kasnije.");

    }
  }
  return WIFI_CONFIGURED;
}

void Watchy::showUpdateFW() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Please visit");
  display.println("watchy.sqfmi.com");
  display.println("with a Bluetooth");
  display.println("enabled device");
  display.println(" ");
  display.println("Press menu button");
  display.println("again when ready");
  display.println(" ");
  display.println("Keep USB powered");
  display.display(false); // full refresh

  guiState = FW_UPDATE_STATE;
}

void Watchy::updateFWBegin() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Bluetooth Started");
  display.println(" ");
  display.println("Watchy BLE OTA");
  display.println(" ");
  display.println("Waiting for");
  display.println("connection...");
  display.display(false); // full refresh

  BLE BT;
  BT.begin("Watchy BLE OTA");
  int prevStatus = -1;
  int currentStatus;

  while (1) {
    currentStatus = BT.updateStatus();
    if (prevStatus != currentStatus || prevStatus == 1) {
      if (currentStatus == 0) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("BLE Connected!");
        display.println(" ");
        display.println("Waiting for");
        display.println("upload...");
        display.display(false); // full refresh
      }
      if (currentStatus == 1) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("Downloading");
        display.println("firmware:");
        display.println(" ");
        display.print(BT.howManyBytes());
        display.println(" bytes");
        display.display(true); // partial refresh
      }
      if (currentStatus == 2) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("Download");
        display.println("completed!");
        display.println(" ");
        display.println("Rebooting...");
        display.display(false); // full refresh

        delay(2000);
        esp_restart();
      }
      if (currentStatus == 4) {
        display.setFullWindow();
        display.fillScreen(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);
        display.setTextColor(GxEPD_WHITE);
        display.setCursor(0, 30);
        display.println("BLE Disconnected!");
        display.println(" ");
        display.println("exiting...");
        display.display(false); // full refresh
        delay(1000);
        break;
      }
      prevStatus = currentStatus;
    }
    delay(100);
  }

  // turn off radios
  WiFi.mode(WIFI_OFF);
  btStop();
  showMenu(menuIndex, false);
}

void Watchy::showSyncNTP() {
  display.setFullWindow();
  display.fillScreen(GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_WHITE);
  display.setCursor(0, 30);
  display.println("Syncing NTP... ");
  display.print("GMT offset: ");
  display.println(gmtOffset);
  display.display(false); // full refresh
  if (connectWiFi()) {
    if (syncNTP()) {
      display.println("NTP Sync Success\n");
      display.println("Current Time Is:");

      RTC.read(currentTime);

      display.print(tmYearToCalendar(currentTime.Year));
      display.print("/");
      display.print(currentTime.Month);
      display.print("/");
      display.print(currentTime.Day);
      display.print(" - ");

      if (currentTime.Hour < 10) {
        display.print("0");
      }
      display.print(currentTime.Hour);
      display.print(":");
      if (currentTime.Minute < 10) {
        display.print("0");
      }
      display.println(currentTime.Minute);
    } else {
      display.println("NTP Sync Failed");
    }
  } else {
    display.println("WiFi Not Configured");
  }
  display.display(true); // full refresh
  delay(3000);
  showMenu(menuIndex, false);
}

bool Watchy::syncNTP() { // NTP sync - call after connecting to WiFi and
                         // remember to turn it back off
  return syncNTP(gmtOffset,
                 settings.ntpServer.c_str());
}

bool Watchy::syncNTP(long gmt) {
  return syncNTP(gmt, settings.ntpServer.c_str());
}

bool Watchy::syncNTP(long gmt, String ntpServer) {
  // NTP sync - call after connecting to
  // WiFi and remember to turn it back off
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, ntpServer.c_str(), gmt);
  timeClient.begin();
  if (!timeClient.forceUpdate()) {
    return false; // NTP sync failed
  }
  tmElements_t tm;
  breakTime((time_t)timeClient.getEpochTime(), tm);
  RTC.set(tm);
  return true;
}
