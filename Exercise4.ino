#include <EEPROM.h>
//a struct for controlling button input
struct Button
{
  private:
    bool consume_release = false;
    bool consume_counting = false;
    int button_state = LOW;
    int button_pin;
    int released = false;
    unsigned long time_pressed = 0;
    unsigned long prev_time = 0;

    //updates the time_pressed
    //which represnts how much time the button is pressed continously
    void updateTimePressed()
    {
      if (button_state == HIGH && prev_time != 0)
      {
        time_pressed = millis() - prev_time;
      }
      else
      {
        prev_time = millis();
        time_pressed = 0;
      }
      if (consume_counting)
      {
        time_pressed = 0;
      }
    }

  public:
    Button(int button_pin): button_pin(button_pin)
    {

    }
    // getTimePressed will return 0 till the button will be released
    void consumeCounting()
    {
      consume_counting = true;
    }
    //init the button pin mode
    void init()
    {
      pinMode(button_pin, INPUT_PULLUP);
    }

    //read the button state
    //and update the release flag (release = true <===> key was pressend and released)
    void read()
    {
      int current_state = !digitalRead(button_pin);
      released = button_state == HIGH && current_state == LOW ;
      consume_counting = current_state == LOW ? false : consume_counting;
      if (released && consume_release)
      {
        released = consume_release = false;
      }
      button_state = current_state;
      updateTimePressed();

    }



    int getState() // return high if button is hold else LOW
    {
      return button_state;
    }

    //return true if the button is preseed
    int isPressed()
    {
      return button_state;
    }

    //return true if the buttson is released after a press!!!
    int isReleased()
    {
      return released;

    }

    //return the amount of time the button is pressed
    unsigned long getTimePressed()
    {
      return time_pressed;
    }

    //ignore the next release
    void consumeRelease()
    {
      consume_release = true;
    }

};

enum class SeminorState
{
  STOP,
  DRIVE_SLOW,
  GO
};

enum class ProgramState
{
  INIT,
  RUNNING
};

//a struct for the logic of the system (states and emergency mode)
struct SeminorLogic
{

  private:
    bool emergency = false;
    bool high_temp = false;
    int current_state = 0;
    SeminorState seminor_states [3];
  public:

    void setEmergency(bool emergency)
    {
      this->emergency = emergency;
    }
    //return the state of the seminor (STOP,SLOW,GO)
    SeminorState getState()
    {
      if (emergency)
      {
        return SeminorState::STOP;
      }
      else if (seminor_states[current_state] == SeminorState::GO && high_temp)
      {
        return SeminorState::DRIVE_SLOW;
      }
      else
      {
        return seminor_states[current_state];
      }
    }

    //return the system state when emergency = false and high_temp = false
    SeminorState getStateWithOutFlags()
    {
      bool save_emergency = emergency;
      bool save_high_temp = high_temp;
      emergency = false;
      high_temp = false;
      SeminorState state_without_flags = getState();
      emergency = save_emergency;
      high_temp = save_high_temp;
      return state_without_flags;
    }
    //returns the index of the current state
    int getStateIndex()
    {
      const int stop_index = 0;
      const int drive_slow_index = 1;
      if (emergency)
      {
        return stop_index;
      }
      else if (high_temp && getState() != SeminorState::STOP)
      {
        return drive_slow_index;
      }
      else
      {
        return current_state;
      }
    }
    bool isEmergency()
    {
      return emergency;
    }

    bool setHighTemp(bool high_temp)
    {
      this->high_temp = high_temp;
    }

    bool isHighTemp() {
      return high_temp;
    }

    //change the state to its  next state by the next order
    // STOP ===> DRIVE SLOW ===> DRIVE ===> STOP
    void changeState()
    {
      int states_num = sizeof(seminor_states) / sizeof(seminor_states[0]);
      current_state = (current_state + 1) % states_num;
    }
    SeminorLogic(): seminor_states{SeminorState::STOP, SeminorState::DRIVE_SLOW, SeminorState::GO} {}

};

struct Matrix //struct for controlling the 8x8 matrix
{
  public:
    static const int MSIZE = 8;
  private:
    int DIN;
    int CS;
    int CLK;

    int getBit(byte b, int bit_index) //  b = (b7,b6,...,b0) return the bit at bit_index of the byte b
    {
      return (b & (1 << bit_index)) != 0;
    }
    byte createColumnByte(byte  matrix [MSIZE], int col)//take the bits at column = col in the matrix and create byte from them
    {
      const int MSB_BIT_INDEX = 7;
      byte column_byte = 0;
      for (int i = MSB_BIT_INDEX; i >= 0; i--)
      {
        column_byte = (column_byte << 1) | getBit(matrix[i], MSB_BIT_INDEX - col);
      }

      return column_byte;
    }


  public:

    void latchBuf() // Latch the entire buffer (sends the entire buffer to the matrix)
    {
      digitalWrite(CS, LOW);
      digitalWrite(CS, HIGH);
    }
    void init()//init the pins of the matrix
    {
      pinMode(DIN, OUTPUT);
      pinMode(CS, OUTPUT);
      pinMode(CLK, OUTPUT);
      writeColumn(0x0C, 0x01);
      writeColumn(0x0B, 0x07);
    }
    Matrix(int DIN, int CS, int CLK): DIN(DIN), CLK(CLK), CS(CS) {}


    //the function turn on/off the leds by the matrix values
    // matrix[i,j] = 1 ===> led[i,j] of the 8x8 matrix is on
    // matrix[i,j] = 0 ===> led[i,j] of the 8x8 is off
    void writeMatrix(byte  matrix [MSIZE])
    {

      for (int i = 0; i < MSIZE; i++)
      {
        writeColumn(i + 1, createColumnByte(matrix, i));
      }

    }


    void clear() // turn off all the matrix leds
    {
      for (int i = 1; i <= MSIZE; i++)
      {
        writeColumn(i, 0);
      }
    }

    void writeBit(bool b) // Write 1 bit to the buffer
    {
      digitalWrite(DIN, b);
      digitalWrite(CLK, LOW);
      digitalWrite(CLK, HIGH);

    }
    void writeColumn(int col, byte value) // write a byte to specific column in the matrix
    {
      writeByte(col);
      writeByte(value);
      latchBuf();
    }
    void writeByte(byte b) //write a byte to the matrix
    {
      for (int i = sizeof(byte) * 8 - 1; i >= 0; i--)
      {
        writeBit(getBit(b, i));
      }
    }

};


/*NUMBERS MATRICES */
byte TEN_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b01001111,
  0b11001001,
  0b01001001,
  0b01001001,
  0b01001111,
  0b00000000
};

byte TWENTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b00101001,
  0b11101001,
  0b10001001,
  0b11101111,
  0b00000000
};

byte THIRTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b00101001,
  0b11101001,
  0b00101001,
  0b11101111,
  0b00000000
};

byte FOURTHY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b10101111,
  0b10101001,
  0b11101001,
  0b00101001,
  0b00101111,
  0b00000000
};

byte FIFTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b10001001,
  0b11101001,
  0b00101001,
  0b11101111,
  0b00000000
};

byte SIXTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b10001001,
  0b11101001,
  0b10101001,
  0b11101111,
  0b00000000
};

byte SEVENTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b00101001,
  0b11101001,
  0b00101001,
  0b00101111,
  0b00000000
};

byte EIGHTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b10101001,
  0b11101001,
  0b10101001,
  0b11101111,
  0b00000000
};

byte NINETY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b11101111,
  0b10101001,
  0b11101001,
  0b00101001,
  0b00101111,
  0b00000000
};
byte HUNDRED_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b10111111,
  0b10101101,
  0b10101101,
  0b10101101,
  0b10111111,
  0b00000000
};

byte HUNDRED_TEN_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b10101111,
  0b10101001,
  0b10101001,
  0b10101001,
  0b10101111,
  0b00000000
};

byte HUNDRED_TWENTY_MATRIX [] =
{
  0b00000000,
  0b00000000,
  0b10110111,
  0b10010101,
  0b10110101,
  0b10100101,
  0b10110111,
  0b00000000
};



//TEMPARTURE
const int  HIGH_TEMPARTURE_THRESHOLD = 30; //temprture > HIGH_TEMPARTURE_THRESHOLD <===> HighTemp Mode
bool messure_temparture = true;
int temparture_timer = 0;

//MATRIX

bool matrix_editable = false; //when true and in driveslow state you can edit the matrix speed
byte*speeds[] = {
  TEN_MATRIX, TWENTY_MATRIX, THIRTY_MATRIX, FOURTHY_MATRIX,
  FIFTY_MATRIX, SIXTY_MATRIX, SEVENTY_MATRIX, EIGHTY_MATRIX,
  NINETY_MATRIX, HUNDRED_MATRIX, HUNDRED_TEN_MATRIX, HUNDRED_TWENTY_MATRIX

};
int num_of_speeds = sizeof(speeds) / sizeof(speeds[0]);
const int SPEED_ROM_ADDRESS = 0;
int speed_index = 0;
int rom_speed_index = 0;
const int high_temp_speed_index  = 7; // 80 speed
bool show_matrix = true; // for blinking the matrix
int matrix_blink_timer = 0;

//PINS
const int mainSeminor_GreenLed = 12;
const int mainSeminor_RedLed = 7;
const int mainSeminor_YellowLed = 8;
const int backSeminor_GreenLed = 5;
const int backSeminor_YellowLed = 6;
const int tempSensor_pin = A1;
const int DIN = 2;
const int CS = 3;
const int CLK = 4;
const int emrgency_led = 10;

//EMERGENCY
int emergency_power = 0;
int emergencyLed_timer  = 0; // for fading the emergency led
int steps = 1;


//DELAYS MS
const int TIME_TO_STOP_EMERGENCY = 2000; //time in ms to hold the emergency button to stop emergency
const int INIT_ON_DELAY = 2000; // the time in ms the leds will be on when the system restart
const int INIT_OFF_DELAY = 1000;// the time in ms the leds will be off after they will be on
const int PROG_DELAY = 4;
const int TIME_TO_STABILIZE_TEMP = 800; //time to wait between temprature reads after start/stop highTemp mode
const int TIME_TO_CHANGE_MATRIX_MODE = 1000; // time in ms to hold the state_button to change the matrix from fixed <===> editable
const int EMERGENCY_LED_BLINK_TIME = 500;
const int MATRIX_BLINK_TIME = 500;

//CONSTANTS
const int HIGH_POWER = 255;
const int ONE_SECOND_MS = 1000;
//LEDS ARRAYS AND LOGIC
ProgramState prog_state = ProgramState::INIT;
SeminorLogic seminor_logic;
int main_seminor_leds [] = {mainSeminor_RedLed, mainSeminor_YellowLed, mainSeminor_GreenLed};
int leds [] = {mainSeminor_RedLed, mainSeminor_YellowLed, mainSeminor_GreenLed, backSeminor_GreenLed, backSeminor_YellowLed, emrgency_led};
//HARDWARE STRUCTS
Button emergency_button(11);
Button changeState_button (9);
Matrix matrix(DIN, CS, CLK);

//TRAIN LOGO
const int LOGO_COLS = 57;
byte TRAIN_FULL_LOGO_MATRIX [Matrix::MSIZE][LOGO_COLS]  = {
  {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

int SCROLL_LOGO_TIME_SEC = 4; // in seconds
const int PAUSE_LOGO_TIME = ONE_SECOND_MS; //ms
bool pause_logo = false;
int logo_firstFrame_offset = LOGO_COLS - 1 - Matrix::MSIZE ;
int logo_offset = logo_firstFrame_offset;
float LOGO_FRAME_CHANGE_RATE = (SCROLL_LOGO_TIME_SEC * ONE_SECOND_MS) / (logo_firstFrame_offset + 1); // logo_firstFrame_offset + 1 is the total number of frames
unsigned long logo_timer = millis();
byte logo_frame_matrix[Matrix::MSIZE];



void setup() {
  Serial.begin(115200);
  for (int led : leds)
  {
    pinMode(led, OUTPUT);
  }
  emergency_button.init();
  changeState_button.init();

  //MATRIX
  matrix.init();
  rom_speed_index = speed_index = loadSpeedFromRom();
  matrix.clear();
}

void loop() {
  if (prog_state == ProgramState::INIT)
  {
    initSystem();
  }
  else
  {
    readButtons();
    checkAndUpdateTempartureMode();
    checkAndUpdateEmergencyMode();
    checkAndUpdateSeminorState(); //handle the functionlity of changing seminor state (STOP==>SLOW===>GO)
    matrixHandler(); // handle all the matrix functionality and states
    emergencyLedHandler(); // handle the states of the emergency led
    displayMatrix(); // display the matrix according the system state and matrix state
    displayLeds(); //display the leds according to the system state
  }
  delay(PROG_DELAY);
}

bool updateAndGetPauseLogo()//when pause_logo = true this function will keep it true till PAUSE_LOGO_TIME passed
{
  bool was_paused = pause_logo;
  if (pause_logo && millis() - logo_timer >= PAUSE_LOGO_TIME)
  {
    pause_logo = false;
  }

  if (was_paused && !pause_logo)
  {
    logo_offset = logo_firstFrame_offset; // begin the scrolling from the first frame
    createCurrentLogoFrame();
  }

  return pause_logo;
}
void handleTrainLogo()//this function handle the logo functionility when in go state
{
  if (seminor_logic.getState() == SeminorState::GO)
  {
    if (updateAndGetPauseLogo())
    {
      return;
    }

    if (millis() - logo_timer >= LOGO_FRAME_CHANGE_RATE)
    {
      changeLogoFrame();
    }

  }
}

void resetLogo() //reset the logo to its begining frame
{
  int prev_offset = logo_offset;
  logo_timer = millis();
  logo_offset = logo_firstFrame_offset;
  pause_logo = false;
  if (prev_offset != logo_offset)
  {
    createCurrentLogoFrame();
  }

}
void changeLogoFrame() // change the logo frame by one frame
{
  logo_offset--;
  if (logo_offset < 0 )
  {
    logo_offset = LOGO_COLS - 1;
  }
  pause_logo = logo_offset == 0;
  logo_timer = millis();
  createCurrentLogoFrame();
}

//creates a logo frame from the TRAIN_FULL_LOGO_MATRIX (the frame is 8x8 matrix that start in column = offset)
void createLogoFrameMatrix(int offset, byte logo_matrix[Matrix::MSIZE])
{
  for (int i = 0; i < 8; i++)
  {
    byte row_i = 0;
    for (int j = logo_offset; j < logo_offset + Matrix::MSIZE; j++)
    {
      row_i = row_i << 1;
      row_i |= TRAIN_FULL_LOGO_MATRIX[i][j % LOGO_COLS];
    }

    logo_matrix[i] = row_i;
  }

}
void createCurrentLogoFrame() //update logo_matrix so it will contain the current frame
{
  createLogoFrameMatrix(logo_offset, logo_frame_matrix);
}

/*---------------------------------------------------------------------------------------------*/

void checkAndUpdateEmergencyMode() //the function update the emrgency flag in the logic according to the emrgency_button presses by the user
{
  if (seminor_logic.isEmergency())
  {
    if (emergency_button.getTimePressed() >=  TIME_TO_STOP_EMERGENCY) //if emergency button pressed over two seconds then stop it
    {
      stopEmergency();
      emergency_button.consumeRelease(); //ignore next release (we dont want invoke emergency after it just stoped)
    }
  }
  else  if (emergency_button.isReleased())
  {
    startEmergency();
  }
}
void checkAndUpdateSeminorState() //this function handle all the functions handle the changeState Button
{
  if (changeState_button.isReleased() && !matrix_editable && !seminor_logic.isEmergency())
  {
    seminor_logic.changeState(); //STOP ==> SLOW ===> DRIVE ===> STOP
    resetLogo();
  }
}


void emergencyLedHandler() // fades emegrency led when emergency and blink it when temp > 30 and we are not in emergency mode
{
  if (seminor_logic.isEmergency())
  {
    fadeEmergencyLed();
  }
  else if (seminor_logic.isHighTemp())
  {
    blinkEmergencyLed();
  }
  else
  {
    emergency_power = 0;
  }

}


void matrixHandler() //this code is relevent to the matrix functionality
{

  if (seminor_logic.getState() == SeminorState::DRIVE_SLOW && !seminor_logic.isHighTemp()) //matrix is  controled only on drive slow and no highTemp
  {

    if (changeState_button.isReleased() && matrix_editable )
    {
      changeMatrixSpeed();
    }

    if (changeState_button.getTimePressed() >= TIME_TO_CHANGE_MATRIX_MODE )
    {
      handleChangeMatrixMode(); //editable,fixed
    }

    if (matrix_editable) // the matrix blinks when you can change its speeds
    {
      blinkMatrix();
    }

  }

  handleTrainLogo();

}

//the function turn on all the leds for two seconds
//and then turn them off for a second
void initSystem()
{
  setLeds(HIGH);
  delay(INIT_ON_DELAY);
  setLeds(LOW);
  delay(INIT_OFF_DELAY );
  prog_state = ProgramState::RUNNING;
}

//update the buttons states and flags (released,pressed)
void readButtons()
{
  emergency_button.read();
  changeState_button.read();
}

//the function update the system
// to be in emrgency mode (turn on the emergency led, set the seminors to stop)
void startEmergency()
{
  seminor_logic.setEmergency(true);
  emergency_power = HIGH_POWER;
  steps = -1;
}

//the function update the emrgency_power
//so it will fade from low to high and opposite
void fadeEmergencyLed()
{
  emergency_power += steps;
  if (emergency_power <= 0 || emergency_power >= HIGH_POWER)
  {
    steps *= -1;
  }
}

//show the leds by the system state
void displayLeds()
{
  displayMainSeminorLeds();
  displayBackSeminorLeds();
  analogWrite(emrgency_led, emergency_power);

}

//display the back seminor leds
// according to the main seminor state
void displayBackSeminorLeds()
{
  switch (seminor_logic.getState())
  {

    case SeminorState::STOP:
      turnOn(backSeminor_YellowLed);
      turnOff(backSeminor_GreenLed);
      break;
    case SeminorState::DRIVE_SLOW:
      turnOn(backSeminor_GreenLed);
      turnOn(backSeminor_YellowLed);
      break;
    case SeminorState::GO:
      turnOn(backSeminor_GreenLed);
      turnOff(backSeminor_YellowLed);
      break;
  }
}

//display the main seminor leds
//according to the state of the main seminor
void displayMainSeminorLeds()
{
  for (int led : main_seminor_leds)
  {
    turnOff(led);
  }

  turnOn(main_seminor_leds[seminor_logic.getStateIndex()]);
}

//the function stop the emergency mode
//and return the system to its state when the emrgency were started
void stopEmergency()
{
  seminor_logic.setEmergency(false);
  emergency_power = 0;
  if (seminor_logic.isHighTemp())
  {
    emergency_power = HIGH_POWER;
    emergencyLed_timer  = 0;
  }

  if (seminor_logic.getState() == SeminorState::DRIVE_SLOW)
  {
    show_matrix = true;
    matrix_blink_timer = 0;
  }
}

//digitalWriting power to all the leds in the leds array
void setLeds(int power)
{
  for (int led : leds)
  {
    digitalWrite(led, power);
  }
}

//turn on a led
void turnOn(int led)
{
  digitalWrite(led, HIGH);
}

//turn off a led
void turnOff(int led)
{
  digitalWrite(led, LOW);
}


//.....................................................................................................
void blinkEmergencyLed() // make the emergency led to blink every EMERGNECY_LED_BLINK_TIME
{
  emergencyLed_timer += PROG_DELAY;
  if (emergencyLed_timer >= EMERGENCY_LED_BLINK_TIME)
  {
    emergencyLed_timer = 0;
    emergency_power = emergency_power != 0 ? 0 : HIGH_POWER;
  }

}

bool isTempStabilize()//this is delay to allow the temparture stabilize after reaching HIGH_TEMP or going under HIGH_TEMP
{

  if (!messure_temparture)
  {
    temparture_timer += PROG_DELAY;
    if (temparture_timer >= TIME_TO_STABILIZE_TEMP)
    {
      temparture_timer = 0;
      messure_temparture = true;
    }
  }

  return messure_temparture;

}

void checkAndUpdateTempartureMode() //check and update the flags if the temparture is greater then TEMPRATURE_THRESHOLD
{
  if (!isTempStabilize())//delay of  TIME_TO_STABILIZE_TEMP
  {
    return; //waiting temprature to "stabilize"
  }
  float temparture = getTemparture();
  if (temparture > HIGH_TEMPARTURE_THRESHOLD && ! seminor_logic.isHighTemp())
  {
    startHighTempMode();
    messure_temparture = false; //this is for isTempStabilize delay
  }
  else if (seminor_logic.isHighTemp() && temparture <= HIGH_TEMPARTURE_THRESHOLD)
  {
    stopHighTempMode();
    messure_temparture = false;

  }
}
void blinkMatrix() // make the matrix blink
{
  matrix_blink_timer += PROG_DELAY;
  if (matrix_blink_timer >= MATRIX_BLINK_TIME)
  {
    matrix_blink_timer = 0;
    show_matrix = !show_matrix;

  }
}
void changeMatrixSpeed() // change the current speed to the next speed by the order in speeds array
{
  speed_index++;
  speed_index %= num_of_speeds;
  matrix_blink_timer = 0; // reset the blinking timer when changing the speed
  show_matrix = true;
}

// flipping the matrix mode (fixed,editable)
//when changed from editable to fixed the speed is saved in the rom
void handleChangeMatrixMode()
{
  changeState_button.consumeCounting();//set getTimePressed to 0 and stop counting time till next release
  changeState_button.consumeRelease();//ignore next release

  matrix_editable = !matrix_editable;
  if (matrix_editable) /// matrix_editable = true ===> matrix will start blink and user can change speeds
  {
    matrix_blink_timer = 0;
  }
  else
  {
    saveSpeedToRom(speed_index);
    rom_speed_index = speed_index;
  }
  show_matrix = !matrix_editable; // imedatly blink when  matrix_editable set to true
}
void displayMatrix()//display or hide the matrix by the system state
{
  if (show_matrix && seminor_logic.getState() == SeminorState::DRIVE_SLOW )
  {
    matrix.writeMatrix(speeds[seminor_logic.isHighTemp() ? getHighTempSpeedFixed() : speed_index]);
  }
  else if (seminor_logic.getState() == SeminorState:: GO)
  {
    matrix.writeMatrix(logo_frame_matrix);
  }
  else
  {
    matrix.clear();
  }
}


// get the speed to show in hightemp by the system state
// GO State ===> 80 speed
// DRIVE_SLOW ==> min(80,speed of drive slow mode)
int getHighTempSpeedFixed()
{
  int high_temp_speed_index_fix = min(rom_speed_index, high_temp_speed_index);
  if (seminor_logic.getStateWithOutFlags() == SeminorState::GO) // if we in go mode then we display high_temp_speed_index which points on 80 speed
  {
    high_temp_speed_index_fix = high_temp_speed_index;
  }

  return high_temp_speed_index_fix;
}
void stopHighTempMode()//the function disable the highTemp mode
{
  if (!seminor_logic.isEmergency())
  {
    emergency_power = 0;
  }
  if (matrix_editable)
  {
    show_matrix = true;
    matrix_blink_timer = 0;
  }
  seminor_logic.setHighTemp(false);
}
void startHighTempMode() //the function set the system state to highTemp
{

  seminor_logic.setHighTemp(true);
  emergencyLed_timer = 0;
  show_matrix = true;
  if (!seminor_logic.isEmergency())
  {
    emergency_power = HIGH_POWER;
  }
}
int loadSpeedFromRom()//return the speed from the rom if valid else return speed_index = 0
{
  const int default_speed_index = 0;
  int speedInRom = EEPROM.read(SPEED_ROM_ADDRESS);
  if (speedInRom < 0 || speedInRom >= num_of_speeds)
  {
    speedInRom = default_speed_index; //default
  }

  return speedInRom;
}

void saveSpeedToRom(int speed) //save speed in the rom at addr = SPEED_ROM_ADDRESS
{
  EEPROM.write(SPEED_ROM_ADDRESS, speed);
}


float getTemparture() //return the temparture in celcios
{
  float R1 = 10000;
  float logR2, R2, temparture, tempartureC;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  int Vo = analogRead(tempSensor_pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  temparture = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  tempartureC = temparture - 273.15;
  return tempartureC;

}
