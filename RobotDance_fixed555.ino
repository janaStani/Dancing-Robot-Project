#include <Servo.h>
#include <EEPROM.h>

// PINS
#define LEFT    12
#define RIGHT   13
#define BUTTON   2
#define LED     11

#define S0  A0   // outer left
#define S1  A1   
#define S2  A2   // centre
#define S3  A3   
#define S4  A4   // outer right

// TUNING CONSTANTS
static int  LINE_TH            = 400;   // sensor threshold
const int   base               = 15;    // forward speed % during line-follow
const float Kp                 = 85.0f; // proportional gain for line correction

const unsigned long TURN_90_MS           = 380;  // ms for one 90° pivot turn
const unsigned long TURN_180_MS          = 600;  // ms for one 180° pivot turn
const unsigned long NOSE_IN_MS           = 290;  // ms to move forward into intersection centre
const unsigned long POST_TURN_NOSE_IN_MS = 190;  // ms to move forward after a turn
const unsigned long ACQUIRE_TIMEOUT_MS   = 1200; // safety timeout for line-acquire after turn
const unsigned long SETTLE_FWD_MS        = 180;  // ms to drive forward after turn before acquire
const unsigned long COMMIT_MS            = 220;  // ms to drive into intersection before stopping
const unsigned long START_IGNORE_MS      = 800;  // ignore intersections right after start
const int           CLEAR_STREAK_TH      = 3;    // consecutive non-intersection sensor readings 

// SERIAL INPUT BUFFER
const int  INPUT_MAX = 220;
static char g_userBuf[INPUT_MAX];        // what the user is currently typing (starts empty)
static int  g_userLen        = 0;
static bool g_userFreshInput = false;    // true = user has started typing something new
static char g_choreoStr[INPUT_MAX];      // choreography currently in use (from EEPROM default/user input)

static int  g_parseErrStreak = 0;
const int   MAX_ERR_STREAK   = 5;        // reset only after this many consecutive errors
static int  g_lastGoodLen    = 0;        // buffer length at last successful parse (for error recovery)

// EEPROM LAYOUT
const uint16_t EEPROM_MAGIC  = 0xD4CE;   // to distinguish valid inputs
const int      EEPROM_ADDR   = 0;        // starting address

// MOTOR CLASS
class Motor : public Servo {
public:
  Motor() : _dir(1), _cur(0) {}          // forward rotation, motor initial state (stopped)

  void setDirection(bool forward) { _dir = forward ? 1 : -1; }  

  void go(int pct) {
    pct = constrain(pct, -100, 100);            // make sure speed doesn't go past -100/100
    const int step = 3;                         // speed only changes by 3% points each loop
    if (pct > _cur) _cur += step;
    if (pct < _cur) _cur -= step;
    _cur = constrain(_cur, -100, 100);
    writeMicroseconds(1500 + _dir * _cur * 3);  // sends pulse: stop/forward/backward, dir flips direction, convert speed into µs
  }

private:
  int _dir;
  int _cur;
};

static Motor leftMotor, rightMotor;             // creates two motor objects

inline void stopMotors() {
  leftMotor.go(0);
  rightMotor.go(0);
}

// COMPASS / GRID
const int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
const int GRID_MIN = 1, GRID_MAX = 9;

// CHOREOGRAPHY DATA
struct Step {                // one target in the dance
  uint8_t  x;                // grid coordinate of target node, stored as numbers
  uint8_t  y;
  bool     rowFirst;         // E1 column first, 3A row first
  uint16_t leaveT10;         // leave time (in tenths of a second)
};

const int MAX_STEPS = 60;
static Step  g_steps[MAX_STEPS];       // array of all parsed steps
static int   g_stepCount = 0;          // how many are actually valid

static int g_startX = 1, g_startY = 1, g_startDir = NORTH;             // home reference for return
static int g_curX   = 1, g_curY   = 1, g_curDir   = NORTH;             // current robot state

static int g_confirmedX = 1, g_confirmedY = 1, g_confirmedDir = NORTH; // updates only when robot physically stops at a node

// PARSE STATUS
enum ParseStatus : uint8_t {
  PS_NONE    = 0,                    // nothing has been parsed yet
  PS_OK      = 1,                    // choreography is complete and valid (start + at least one step)
  PS_WARNING = 2,                    // incomplete input – keep typing
  PS_ERROR   = 3                     // malformed input
};
static ParseStatus g_parseStatus = PS_NONE;                 // status of currently active choreography
static ParseStatus parseChoreography(const char *s);        // forward declaration

// ROBOT STATE MACHINE
enum RobotState : uint8_t {
  ST_IDLE,         // waiting for first button press to start dance
  ST_DANCE_DONE,   // dance complete, waiting for button press to return to start
  ST_FOLLOW,       
  ST_INTER_COMMIT,
  ST_AT_NODE,
  ST_TURNING,
  ST_ACQUIRE_FWD,
  ST_LEAVE_INTER,
  ST_RETURN
};
static RobotState g_state = ST_IDLE;                    // define robot behavior

// Dance runtime
static bool          g_danceRunning     = false;        // performing the dance
static bool          g_returnRequested  = false;        // go back to start
static unsigned long g_danceStartMs     = 0;            // timestamp since dance started
static int           g_stepIndex        = 0;            // index of current choreography step

static int           g_targetX          = 1;            // target node information
static int           g_targetY          = 1;
static bool          g_targetRowFirst   = false;
static unsigned long g_targetLeaveMs    = 0;

static int  g_turnAmount   = 0;                         // path planner
static int  g_neededDir    = NORTH;
static bool g_pendingTurn  = false;

static bool          g_needClearFirst   = true;
static unsigned long g_ignoreInterUntil = 0;
static int           g_clearStreak      = 0;
static bool          g_retFinalTurn     = false;

// BUTTON DEBOUNCE
static bool          g_btnLastStable  = HIGH;
static bool          g_btnLastReading = HIGH;
static unsigned long g_btnLastChange  = 0;             // timestamp of last raw state change
const unsigned long  BTN_DEBOUNCE_MS  = 30;            // waits for signal to be stable for 30 ms

static bool buttonPressedEdge() {
  bool reading = digitalRead(BUTTON);
  unsigned long now = millis();
  if (reading != g_btnLastReading) {
    g_btnLastReading = reading;
    g_btnLastChange  = now;
  }
  if ((now - g_btnLastChange) >= BTN_DEBOUNCE_MS && reading != g_btnLastStable) {
    g_btnLastStable = reading;
    if (g_btnLastStable == LOW) return true;
  }
  return false;
}

// SENSORS
static void readSensors(int &s0, int &s1, int &s2, int &s3, int &s4) {
  s0 = 1023 - analogRead(S0);            // read sensors analog, get raw values 0 to 1023
  s1 = 1023 - analogRead(S1);            // black = high  
  s2 = 1023 - analogRead(S2);
  s3 = 1023 - analogRead(S3);
  s4 = 1023 - analogRead(S4);
}

// Call once at startup while robot is ON a line crossing, reads all sensors and sets LINE_TH 
static void calibrateSensors() {
  Serial.println(F("Calibrating sensors... place robot on a line crossing."));
  delay(1000);

  int minVal = 1023, maxVal = 0;
  for (int i = 0; i < 50; i++) {                  // sample 50 times
    int s0, s1, s2, s3, s4;
    readSensors(s0, s1, s2, s3, s4);
    int vals[5] = {s0, s1, s2, s3, s4};
    for (int j = 0; j < 5; j++) {                 // scan all sensors
      if (vals[j] < minVal) minVal = vals[j];     // whitest surface detected so far
      if (vals[j] > maxVal) maxVal = vals[j];     // blackest surface detected so far
    }
    delay(10);
  }
  LINE_TH = minVal + (maxVal - minVal) * 4 / 10;  // Set threshold at 40% from min toward max
  if (LINE_TH < 150) LINE_TH = 150;               // Prevents threshold from being too low
  if (LINE_TH > 700) LINE_TH = 700;               // Prevents threshold from being too high
}

// intersection detection: at least 3 sensors active OR centre + outer
static bool isIntersectionRaw(int s0, int s1, int s2, int s3, int s4) {
  bool b0 = s0 > LINE_TH, b1 = s1 > LINE_TH, b2 = s2 > LINE_TH,    // b0 b1 b2 b3 b4 -> 1  1  1  1  0
       b3 = s3 > LINE_TH, b4 = s4 > LINE_TH;
  int cnt = (int)b0 + b1 + b2 + b3 + b4;
  return (cnt >= 3) || (b2 && (b0 || b4));
}

static bool isIntersectionDebounced(int s0, int s1, int s2, int s3, int s4) {
  static bool latched = false;                                     // current intersection state
  static int  onCnt = 0, offCnt = 0;                               // how many times we saw intersection or we didn't
  bool raw = isIntersectionRaw(s0, s1, s2, s3, s4);
  if (raw) { onCnt++; offCnt = 0; } else { offCnt++; onCnt = 0; }
  if (!latched && onCnt  >= 1) latched = true;
  if ( latched && offCnt >= 3) latched = false;                    // 3 consecutive non-intersection readings, to declare that it left intersection
  return latched;
}

// Call before any new navigation to flush stale debouncer state
// (the static vars inside isIntersectionDebounced survive across states)
static void resetIntersectionDebouncer() {
  for (int i = 0; i < 8; i++) {
    isIntersectionDebounced(0, 0, 0, 0, 0);                        // feed zeros → unlatch
    delay(5);
  }
}

// P controller for line following 
static void followLine(int s1, int s2, int s3) {        // uses inner 3 sensors
  int sum = s1 + s2 + s3;                               
  if (sum < 30) sum = 30;                               
  float error      = float(s1 - s3) / sum;              
  int   correction = int(Kp * error);                   // proportional correction
  leftMotor.go( base - correction);                     // correction steers robot toward line center
  rightMotor.go(base + correction);
}

// POSITION TRACKING
static void advancePose() {
  int nx = g_curX, ny = g_curY;                         // copy current position, so it can compute new position safely before committing it
  if      (g_curDir == NORTH) ny++;                     // if facing north, row increases
  else if (g_curDir == SOUTH) ny--;                     // if facing south, row decreases
  else if (g_curDir == EAST)  nx++;                     // if facong east, column increases
  else if (g_curDir == WEST)  nx--;                     // if facing west, column decreases
  if (nx >= GRID_MIN && nx <= GRID_MAX) g_curX = nx;    // only update if new value is inside legal grid
  if (ny >= GRID_MIN && ny <= GRID_MAX) g_curY = ny;     
}

static void printPose() {                               // print robots current grid position and orientation
  char col = 'A' + g_curX - 1;                          // convert into letter
  Serial.print(col);                                    // A..
  Serial.print(g_curY);                                 // 1..
  Serial.print(F(" facing ")); 
  const char *dirs[] = {"N","E","S","W"};               // match direction encoding
  Serial.print(dirs[g_curDir]);
}

// PATH PLANNING
static void calculatePath(int tX, int tY, bool rowFirst) {   // takes target grid coordinates
  if (tX == g_curX && tY == g_curY) {
    g_turnAmount = 0;
    g_neededDir  = g_curDir;
    return;
  }

  int dx = tX - g_curX;                                     // compute displacement, + (positive) target is east, - west
  int dy = tY - g_curY;                                     // + north, - south

  int desired = g_curDir;
  if (!rowFirst) {                            // match inputs, column-first (e.g. E1)
    if      (dx > 0) desired = EAST;
    else if (dx < 0) desired = WEST;
    else if (dy > 0) desired = NORTH;
    else             desired = SOUTH;
  } else {                                    // match inputs, row-first (e.g. 2A)
    if      (dy > 0) desired = NORTH;
    else if (dy < 0) desired = SOUTH;
    else if (dx > 0) desired = EAST;
    else             desired = WEST;
  }

  g_neededDir = desired;

  int diff = ((desired - g_curDir) + 4) % 4;   // calc the direction difference
  if      (diff == 0) g_turnAmount =  0;       // no tun
  else if (diff == 1) g_turnAmount =  1;       // right
  else if (diff == 2) g_turnAmount =  2;       // u-turn
  else                g_turnAmount = -1;       // left

  Serial.print(F("Plan: "));
  printPose();
  Serial.print(F(" -> "));
  char col = 'A' + tX - 1;
  Serial.print(col); Serial.print(tY);
  Serial.print(F("  turn="));
  Serial.println(g_turnAmount);
}

// TURN EXECUTION
static void executeTurn(int s0, int s1, int s2, int s3, int s4) {    // can remove mby parameters cuz not used
  static unsigned long turnStart  = 0;                               // timestamp of current turn
  static int           stepsLeft  = 0;            
  static int           turnDir    = 1;

  if (stepsLeft == 0) {                                              // initialize new turn, we're not in the middle of multi-step turn
    if (g_turnAmount == 0) { g_state = ST_LEAVE_INTER; return; }     // no turn needed
    stepsLeft = (g_turnAmount == 2) ? 2 : 1;                         // u-turn else 90 deg
    turnDir   = (g_turnAmount == -1) ? -1 : 1;                       // left/right
    turnStart = millis();                                            // record start time of step
    Serial.print(F("Turn: "));
    Serial.println(stepsLeft == 2 ? F("U-turn") : (turnDir == 1 ? F("RIGHT") : F("LEFT")));
  }
  // Use TURN_180_MS for U-turn steps, TURN_90_MS for normal 90° steps
  unsigned long thisTurnMs = (stepsLeft == 2 && g_turnAmount == 2) ? TURN_180_MS : TURN_90_MS;

  if (millis() - turnStart < thisTurnMs) {                           // still turning
    if (turnDir == 1) { leftMotor.go(24); rightMotor.go(-25); }      // right turn
    else              { leftMotor.go(-24); rightMotor.go(25); }      // left turn
  } else {                                                           // turn time done
    stopMotors();                                                    // stop motor
    if (turnDir == 1) g_curDir = (g_curDir + 1) % 4;                 // update direction, right turn: N, E, S, W
    else              g_curDir = (g_curDir + 3) % 4;                 // left turn: N, W, S, E

    stepsLeft--;                                                     // decrease remaning steps
    if (stepsLeft > 0) {                                             // if more steps are needed u-turn case
      turnStart = millis();
      Serial.println(F("Next 90°..."));
    } else {                                                         // turn is fully done
      g_turnAmount = 0;
      delay(70);                                                     // small pause
      leftMotor.go(18); rightMotor.go(18);                           // move forward a bit
      delay(SETTLE_FWD_MS);
      stopMotors();
      Serial.println(F("Turn complete."));
      g_state = ST_ACQUIRE_FWD;                                      // switch state to find line
    }
  }
}

// ACQUIRE FORWARD – recover line after a turn
static void acquireForward(bool inter, int s1, int s2, int s3) {
  static bool          active   = false;                            
  static unsigned long t0       = 0;
  static unsigned long noseT0   = 0;
  static bool          noseDone = false;

  if (!active) {                                                    // ran only once when entering state
    active   = true; 
    t0       = millis();
    noseT0   = millis();
    noseDone = false;
    Serial.println(F("Acquire forward..."));
  }

  if (millis() - t0 > ACQUIRE_TIMEOUT_MS) {                        // aquisition takes too long, something went wrong
    stopMotors(); active = false;
    Serial.println(F("Acquire timeout."));
    g_state = ST_LEAVE_INTER;                                      // try moving on safely
    return;
  }

  if (!noseDone) {                                                 // drive forward briefly
    if (millis() - noseT0 < POST_TURN_NOSE_IN_MS) {
      leftMotor.go(18); rightMotor.go(18);                         // ignore sensors 
      return;
    }
    noseDone = true;                                               // start line following
  }

  followLine(s1, s2, s3); 

  if (!inter) {                                                    // exit condition, cleanly on a line
    stopMotors(); active = false;
    Serial.println(F("Acquire done."));
    if (g_retFinalTurn) {                                          // if final turn during return, skip ST_FOLLOW
      g_state = ST_RETURN;
    } else {
      g_state = ST_FOLLOW;
    }
  }
}

// INTERSECTION COMMIT
static void doIntersectionCommit() {                              // commit to the intersection center
  static bool          active = false;
  static unsigned long t0     = 0;

  if (!active) { active = true; t0 = millis(); }                  // ran only once when entering  state

  if (millis() - t0 < NOSE_IN_MS) {                               // drive forward a bit
    leftMotor.go(20); rightMotor.go(20);
    return;
  }

  stopMotors(); active = false;
  g_state = ST_AT_NODE;                                           // robot is centered
}

// LEAVE INTERSECTION
static void doLeaveIntersection(bool inter, int s1, int s2, int s3) {    
  static int clearStreak = 0;                                    // how many consecutive loops the robot sees no intersecion

  if (inter) {                                                   // still on intersectiton
    clearStreak = 0;                                             // reset counter
    leftMotor.go(15); rightMotor.go(15);                         // drive forward, no line following yet
    return;
  }

  followLine(s1, s2, s3);                                        // no intersection, normal line
  clearStreak++;

  if (clearStreak >= CLEAR_STREAK_TH) {                          // require multiple clear readings
    stopMotors(); clearStreak = 0;
    g_needClearFirst = false;                                    // allow future intersection detection
    g_state = ST_FOLLOW;                                         // switch to normal line following
  }
}

// AT NODE decision making while at intersection
static void handleAtNode() {
  stopMotors();                                                  // stop motion

  g_confirmedX   = g_curX;                                       // update confirmed position only when fully stopped at node
  g_confirmedY   = g_curY;
  g_confirmedDir = g_curDir;

  if (g_returnRequested) {                                       // if in return mode
    g_state = ST_RETURN;                                         // switch to return logic, do not continue with choregraphy
    return;
  }

  if (g_stepIndex >= g_stepCount) {                              // if dance finished, no more steps
    Serial.println(F("Dance complete! Press button to return to start."));
    digitalWrite(LED, LOW);
    g_danceRunning = false;
    g_state = ST_DANCE_DONE;
    return;
  }

  if (g_curX == g_targetX && g_curY == g_targetY) {             // only wait if we are at the target
    unsigned long elapsed = millis() - g_danceStartMs;

    if (elapsed < g_targetLeaveMs) {                            // have we reached the absolute leave time for this step yet?         
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 1000) {                        // print waiting message once per second
        Serial.print(F("Waiting at ")); 
        printPose();
        Serial.print(F("  "));
        Serial.print(elapsed / 1000);
        Serial.print(F("/"));
        Serial.print(g_targetLeaveMs / 1000);
        Serial.println(F("s"));
        lastPrint = millis();
      }
      if (buttonPressedEdge()) {                               // button press during waiting triggers return to start
        g_returnRequested = true;
        g_state = ST_RETURN;
        Serial.println(F("Return requested."));
      }
      return;
    }

    Serial.print(F("Leaving "));                              // if time has passed, advance to next step
    printPose();
    Serial.println();
    g_stepIndex++;

    if (g_stepIndex >= g_stepCount) {                         // if that was the last step, finish dance
      Serial.println(F("Dance complete! Press button to return to start."));
      digitalWrite(LED, LOW);
      g_danceRunning = false;
      g_state = ST_DANCE_DONE;     
      return;
    }

    g_targetX        = g_steps[g_stepIndex].x;                // load next target step
    g_targetY        = g_steps[g_stepIndex].y;
    g_targetRowFirst = g_steps[g_stepIndex].rowFirst;
    g_targetLeaveMs  = (unsigned long)g_steps[g_stepIndex].leaveT10 * 100UL;    // convert to ms
  }

  calculatePath(g_targetX, g_targetY, g_targetRowFirst);                        // plan toward target
  g_state = (g_turnAmount != 0) ? ST_TURNING : ST_LEAVE_INTER;                  // if turn is needed go to ST_TURNING, else leave intersection
}


// RETURN TO START  
enum RetPhase : uint8_t {
  RET_INIT,                      // first entry, compute plan
  RET_DRIVE_X,                   // driving along X axis
  RET_DRIVE_Y,                   // driving along Y axis
  RET_FACE_HOME,                 // final heading correction
  RET_DONE
};
static RetPhase  g_retPhase    = RET_INIT;      // current phase of return process
static int       g_retDestX    = 0;             // temp coordinates to reach during return
static int       g_retDestY    = 0;
static int       g_retFinalDir = NORTH;         // store original start facing direction

// Called from ST_RETURN every loop tick, decides which existing motion state to use next
static void doReturnToStart() {
  if (g_retPhase == RET_INIT) {                 // runs once, when return mode starts
    g_danceRunning = false;                     // ensures ni choreography logic interferes
    
    g_curX   = g_confirmedX;                    // use the last confirmed node position
    g_curY   = g_confirmedY;
    g_curDir = g_confirmedDir;
    
    Serial.print(F("Returning to start from "));
    printPose();
    Serial.println();

    g_retFinalDir = g_startDir;                 // robot must end facing exactly the same direction it started

    int dx = g_startX - g_curX;                 // positive - east, negative - west
    int dy = g_startY - g_curY;                 // positive - north, negative - south

    if (dx != 0) {                              // robot is not in correct column
      g_retDestX = g_startX;
      g_retDestY = g_curY;                      // X-axis movement, keep current Y
      g_retPhase = RET_DRIVE_X;
    } else if (dy != 0) {                       // robot is not in correct row
      g_retDestX = g_startX;
      g_retDestY = g_startY;                    // Y-axis movement, keep current X
      g_retPhase = RET_DRIVE_Y;
    } else {
      g_retPhase = RET_FACE_HOME;               // already at start cell
    }
  }

  // X axis (column) part, move east/west
  if (g_retPhase == RET_DRIVE_X) {             // X leg complete, start Y leg if needed
    if (g_curX == g_retDestX) {                // do we need Y move
      int dy = g_startY - g_curY;              // robot must move Y
      if (dy != 0) {
        g_retDestY = g_startY;
        g_retPhase = RET_DRIVE_Y;              // fall through to DRIVE_Y 
      } else {
        g_retPhase = RET_FACE_HOME;            // robots already in corrct row, fall through to FACE_HOME
      }
    } else {                                   // travel along columns until correct X
      
      // the standard ST_TURNING / ST_LEAVE_INTER / ST_FOLLOW pipeline
      g_targetX        = g_retDestX;
      g_targetY        = g_retDestY;
      g_targetRowFirst = false;                // column first (X leg)
      g_targetLeaveMs  = 0;                    // dont wait at nodes during return

      calculatePath(g_targetX, g_targetY, g_targetRowFirst);     // is a turn needed before going into next cell?

      g_needClearFirst = true;                                   // ensures clean intersection logic for the next movement
      g_clearStreak    = 0;
      resetIntersectionDebouncer();

      g_state = (g_turnAmount != 0) ? ST_TURNING : ST_FOLLOW;
      return;
    }
  }

  // Y axis (row) part, move south/north
  if (g_retPhase == RET_DRIVE_Y) {
    if (g_curY == g_startY && g_curX == g_startX) {             // already at the start cell
      g_retPhase = RET_FACE_HOME;                               // next face original direction
    } else {                                                    // otherwise, drive toward the start cell
      g_targetX        = g_startX;
      g_targetY        = g_startY;
      g_targetRowFirst = true;                                  // row first (Y leg)
      g_targetLeaveMs  = 0;                                     // no waiting at nodes

      calculatePath(g_targetX, g_targetY, g_targetRowFirst);    // turn before moving north/south?

      g_needClearFirst = true;
      g_clearStreak    = 0;
      resetIntersectionDebouncer();
      g_returnRequested = true;
      g_state = (g_turnAmount != 0) ? ST_TURNING : ST_FOLLOW;
      return;
    }
  }

  // Make correct direction of start cell
  if (g_retPhase == RET_FACE_HOME) {
    if (g_curDir != g_retFinalDir) {                           // wrong direction
      int diff = ((g_retFinalDir - g_curDir) + 4) % 4;         // how far to rotate
      if      (diff == 1) g_turnAmount =  1;
      else if (diff == 3) g_turnAmount = -1;
      else                g_turnAmount =  2;               

      g_neededDir    = g_retFinalDir;             
      g_retPhase     = RET_DONE;                               // after this final heading correction, we’re done returning
      g_retFinalTurn = true;                                   // go back to ST_RETURN instead of driving away
      g_state = ST_TURNING;                                    // switches to turning state to perform the rotation
      return;
    }
    g_retPhase = RET_DONE;                                     // no turn is needed directly mark return phase as done
  }

  // done
  if (g_retPhase == RET_DONE) {
    g_curX = g_startX; g_curY = g_startY; g_curDir = g_startDir;
    g_confirmedX = g_startX; g_confirmedY = g_startY; g_confirmedDir = g_startDir;
    g_returnRequested = false;                                // return mode officially over
    g_retFinalTurn    = false;                                // next turns behave normally
    g_retPhase = RET_INIT; 
    stopMotors();
    Serial.println(F("Returned to start. Press button to dance again."));
    g_state = ST_IDLE;
  }
}


// PARSER
static bool isSeparator(char c) {     // sepparators between tokens space, tab, newline, comma, semicolon
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == ',' || c == ';';
}
static char toUpper(char c) { return (c >= 'a' && c <= 'z') ? c - 32 : c; }
static bool isCoordLetter(char c) { c = toUpper(c); return c >= 'A' && c <= 'I'; }
static bool isCoordDigit(char c)  { return c >= '1' && c <= '9'; }
static bool isAnyDigit(char c)    { return c >= '0' && c <= '9'; }          // for time values
static int  letterToX(char c)     { return toUpper(c) - 'A' + 1; }
static int  digitToY(char c)      { return c - '0'; }

static void skipSep(const char *s, int &i) {   //  move over sepparators
  while (s[i] && isSeparator(s[i])) i++;
}

// Wrong input
static bool parseError(const char *s, int idx, const __FlashStringHelper *msg, char found) {
  g_parseStatus = PS_ERROR;                   // used when wrong and cannot become valid by typing more
  Serial.print(F("Parse ERROR at index ")); Serial.print(idx);
  Serial.print(F(": ")); Serial.println(msg);
  Serial.print(F("  Found: '"));
  if (found == '\0') Serial.print(F("<end>")); else Serial.print(found);
  Serial.println('\'');
  Serial.print(F("  Input: ")); Serial.println(s);
  return false;
}
static bool parseWarn(const char *s, int idx, const __FlashStringHelper *msg) {
  g_parseStatus = PS_WARNING;                // used when input is incomplete, but could be valid if the user keeps typing.
  Serial.print(F("Parse INCOMPLETE at index ")); Serial.print(idx);
  Serial.print(F(": ")); Serial.println(msg);
  return false;
}

//Start token form
static bool parseStart(const char *s, int &i, int &x, int &y, int &dir) {
  if (!s[i]) return parseWarn(s, i, F("expected start token (e.g. A1N)"));
  char c1 = s[i], c2 = s[i+1], c3 = s[i+2];
  if (!isCoordLetter(c1))
    return parseError(s, i,   F("start must begin with a letter A..I."), c1);
  if (!c2)
    return parseWarn(s, i+1,  F("start incomplete - need digit after letter."));
  if (!isCoordDigit(c2))
    return parseError(s, i+1, F("start must have a digit 1..9 after the letter."), c2);
  if (!c3)
    return parseWarn(s, i+2,  F("start incomplete - need direction N/E/S/W."));
  switch (toUpper(c3)) {
    case 'N': dir = NORTH; break;
    case 'E': dir = EAST;  break;
    case 'S': dir = SOUTH; break;
    case 'W': dir = WEST;  break;
    default:  return parseError(s, i+2, F("direction must be N, E, S or W."), c3);
  }
  x  = letterToX(c1);
  y  = digitToY(c2);
  i += 3;
  if (s[i] && !isSeparator(s[i]))
    return parseError(s, i, F("separator expected after start token."), s[i]);
  return true;
} // <Letter A..I><Digit 1..9><Dir N/E/S/W> e.g A1N


//Steps, each step has this form
static bool parseCoord(const char *s, int &i, int &x, int &y, bool &rowFirst) {
  char c1 = s[i], c2 = s[i+1];
  if (!c1) return parseWarn(s, i, F("expected coordinate."));
  if (isCoordLetter(c1)) {     // e.g. E1
    if (!c2)               return parseWarn(s, i+1, F("coordinate incomplete."));
    if (!isCoordDigit(c2)) return parseError(s, i+1, F("letter-coord needs digit 1..9 after letter."), c2);
    x = letterToX(c1); y = digitToY(c2); rowFirst = false; i += 2; return true;
  }
  if (isCoordDigit(c1)) {      // e.g. 3A
    if (!c2)                return parseWarn(s, i+1, F("coordinate incomplete."));
    if (!isCoordLetter(c2)) return parseError(s, i+1, F("digit-coord needs letter A..I after digit."), c2);
    x = letterToX(c2); y = digitToY(c1); rowFirst = true;  i += 2; return true;
  }
  return parseError(s, i, F("invalid coordinate (expected letter A..I or digit 1..9)."), c1);
}


// time token
static bool parseTime(const char *s, int &i, uint16_t &tenths) {
  if (toUpper(s[i]) != 'T')
    return parseError(s, i, F("expected 'T' to start time token."), s[i]);
  i++;
  if (!s[i])              return parseWarn(s, i, F("expected digit(s) after 'T'."));
  if (!isAnyDigit(s[i]))  return parseError(s, i, F("expected digit(s) after 'T'."), s[i]);
  unsigned long val = 0;
  int digStart = i;
  while (s[i] && isAnyDigit(s[i])) { val = val * 10UL + (s[i] - '0'); i++; }
  if (s[i] && !isSeparator(s[i]))
    return parseError(s, i, F("separator expected after time value."), s[i]);
  if (val > 65535UL)
    return parseError(s, digStart, F("time value too large (max 65535)."), s[digStart]);
  tenths = (uint16_t)val;
  return true;
}


// MAIN PARSER  
static ParseStatus parseChoreography(const char *s) {    // take full choreography, return status
  g_stepCount   = 0;                   // reset state
  g_parseStatus = PS_NONE;     

  int i = 0;
  skipSep(s, i);                      // skip sepparators
  if (!s[i]) return parseWarn(s, i, F("empty input.")), g_parseStatus;    // empty input check

  int tmpX, tmpY, tmpDir;             // parse start token
  if (!parseStart(s, i, tmpX, tmpY, tmpDir)) return g_parseStatus;   // expects Letter Digit Direction, if fails set status and print message
  g_startX = tmpX; g_startY = tmpY; g_startDir = tmpDir;             // if okay store parsed token

  // pair parse
  bool expectCoord = true;              // next token should be a coordinate
  int  pendX = 0, pendY = 0;
  bool pendRowFirst = false;            // cou can’t store a step until you have both coord + time
  bool hasPend = false;

  while (true) {                        // keep reading tokens
    skipSep(s, i);                      // move past sepparators
    if (!s[i]) {                       // if end of string
      if (!expectCoord) {              // We have a coordinate but no time token yet – incomplete
        parseWarn(s, i, F("incomplete: coordinate present but time token missing."));
        return g_parseStatus;
      }
      // if expectCoord == true
      // if we have zero steps, that means only the start token was given – still incomplete.
      if (g_stepCount == 0) {
        parseWarn(s, i, F("incomplete: no waypoints yet (need at least one coord + time)."));
        return g_parseStatus;
      }
      // At least one step exists and we are between steps: fully valid.
      break;
    }

    // expect the next token to be a coordinate
    if (expectCoord) {
      if (!parseCoord(s, i, pendX, pendY, pendRowFirst)) return g_parseStatus;   // coordinate 
      hasPend = true; expectCoord = false;
    } else {                       // expecting time token
      skipSep(s, i);
      if (!s[i]) {
        parseWarn(s, i, F("incomplete: coordinate without time token."));
        return g_parseStatus;
      }
      if (toUpper(s[i]) != 'T')  // If the next non-separator character isn’t T, it’s a hard ERROR
        return parseError(s, i, F("expected 'T' after coordinate."), s[i]), g_parseStatus;

      uint16_t t10;
      if (!parseTime(s, i, t10)) return g_parseStatus;

      if (!hasPend) return parseError(s, i, F("internal error: no pending coord."), s[i]), g_parseStatus;
      if (g_stepCount >= MAX_STEPS)
        return parseError(s, i, F("too many steps (max 60)."), s[i]), g_parseStatus;

      g_steps[g_stepCount++] = { (uint8_t)pendX, (uint8_t)pendY, pendRowFirst, t10 };   // stores the full step
      hasPend = false; expectCoord = true;
    }
  }

  // Require at least one waypoint – a start-only string like "A1N" is still incomplete
  if (g_stepCount == 0) {
    parseWarn(s, i, F("incomplete: no waypoints yet (need at least one coord + time)."));
    return g_parseStatus;
  }

  g_parseStatus = PS_OK;
  return g_parseStatus;
}


// EEPROM, save last valid choreography, so robot remembers it after power-off and reload it on next step
static void saveToEEPROM(const char *str, int len) {
  uint16_t magic = EEPROM_MAGIC;                                             // the signature, file header
  uint16_t l = (uint16_t)len;
  EEPROM.put(EEPROM_ADDR, magic);
  EEPROM.put(EEPROM_ADDR + 2, l);                                            // string length
  for (int i = 0; i < len; i++)                                              // store actual string
    EEPROM.update(EEPROM_ADDR + 4 + i, (uint8_t)str[i]);
  EEPROM.update(EEPROM_ADDR + 4 + len, 0);
  Serial.println(F("Choreography saved to EEPROM."));
}

static bool loadFromEEPROM() {                                               // try to load previously saved choreography
  uint16_t magic, len;
  EEPROM.get(EEPROM_ADDR, magic);           
  EEPROM.get(EEPROM_ADDR + 2, len);
  if (magic != EEPROM_MAGIC || len == 0 || len >= INPUT_MAX) return false;   // if any check fails ignore EEPROM
  for (int i = 0; i < (int)len; i++)                                         // read chars, in active choreography buffer
    g_choreoStr[i] = (char)EEPROM.read(EEPROM_ADDR + 4 + i);
  g_choreoStr[len] = '\0';
  return true;
}

// DEFAULT CHOREOGRAPHY
static const char DEFAULT_CHOREO[] PROGMEM = "A1N E1T150 B2T350 3AT450 4CT567 D2T700";
 
static void loadDefaultChoreography() {                                  // called when EEPROM is empty
  strncpy_P(g_choreoStr, DEFAULT_CHOREO, INPUT_MAX - 1);                 // copy to ram
  g_choreoStr[INPUT_MAX - 1] = '\0';
  ParseStatus ps = parseChoreography(g_choreoStr);
  if (ps == PS_OK) {
    Serial.println(F("Default choreography loaded."));
  } else {
    Serial.println(F("WARNING: default choreography parse failed!"));
  }
}

// START DANCE
static void startDance() {
  if (g_parseStatus != PS_OK || g_stepCount == 0) {                      
    Serial.println(F("Cannot start: no valid choreography loaded."));
    return;
  }
  Serial.println(F("=== DANCE START ==="));
  digitalWrite(LED, HIGH);

  g_curX   = g_startX; g_curY = g_startY; g_curDir = g_startDir;          // reset pose to starting position
  g_confirmedX = g_startX; g_confirmedY = g_startY; g_confirmedDir = g_startDir;

  g_danceStartMs    = millis();                                     
  g_stepIndex       = 0;
  g_danceRunning    = true;
  g_returnRequested = false;

  g_targetX        = g_steps[0].x;                                        // target cell
  g_targetY        = g_steps[0].y;
  g_targetRowFirst = g_steps[0].rowFirst;
  g_targetLeaveMs  = (unsigned long)g_steps[0].leaveT10 * 100UL;

  g_ignoreInterUntil = millis() + START_IGNORE_MS;                        // ignore intersection after start
  g_needClearFirst   = true;
  g_clearStreak      = 0;
  g_returnRequested  = false;
  g_retPhase         = RET_INIT;
  g_retFinalTurn     = false;

  calculatePath(g_targetX, g_targetY, g_targetRowFirst);                   // plan movement
  g_state = (g_turnAmount != 0) ? ST_TURNING : ST_LEAVE_INTER;             // need turn or alreasy facing correct direction
}


// SERIAL INPUT
static void userBufReset() {
  g_userLen        = 0;
  g_userBuf[0]     = '\0';
  g_userFreshInput = false;
  g_parseErrStreak = 0;
  g_lastGoodLen    = 0;   // full reset clears the good-length checkpoint too
}

// Trim buffer back to last known good position (on error recovery)
static void trimToLastGood() {
  g_userLen        = g_lastGoodLen;
  g_userBuf[g_userLen] = '\0';
  g_userFreshInput = (g_userLen > 0);  // stay in session if there is still content
}

// '!': full wipe, start completely fresh
static void clearBuf() {
  userBufReset();
  Serial.println(F("[Buffer cleared - type full choreography, e.g.: A1N E1T150 B2T350]"));
}

static void readSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // '!' = full clear at any time
    if (c == '!') {
      clearBuf();
      return;
    }

    bool isNewline = (c == '\n' || c == '\r');

    // First character after a full reset: start a new session
    if (!g_userFreshInput) {
      if (isNewline) return;    // ignore stray newlines when idle
      g_userLen     = 0;
      g_userBuf[0]  = '\0';
      g_lastGoodLen = 0;
      g_userFreshInput  = true;
      g_parseErrStreak  = 0;
      Serial.println(F("[Typing started - format: A1N E1T150 B2T350 ... | '!' to clear]"));
    }

    if (isNewline) {
      // ENTER pressed: validate what we have
      if (g_userLen == 0) return;

      Serial.print(F("Buffer: ")); Serial.println(g_userBuf);

      ParseStatus ps = parseChoreography(g_userBuf);

      if (ps == PS_OK) {
        // ── Valid so far: save, checkpoint, but KEEP the buffer so user can add more steps ──
        g_lastGoodLen = g_userLen;   // mark this as the safe rollback point

        strncpy(g_choreoStr, g_userBuf, INPUT_MAX - 1);
        g_choreoStr[INPUT_MAX - 1] = '\0';
        saveToEEPROM(g_userBuf, g_userLen);

        Serial.print(F("OK - start=("));
        Serial.print(char('A' + g_startX - 1)); Serial.print(g_startY);
        Serial.print(F(") steps=")); Serial.print(g_stepCount);
        Serial.println(F("  (keep typing to add more steps, or press BUTTON to start)"));

      } else if (ps == PS_WARNING) {
        // Incomplete but not wrong – keep buffer, wait for more input
        g_parseErrStreak = 0;
        Serial.println(F("(incomplete - keep typing, press Enter when done)"));

      } else {
        // ── Hard error: trim back to last checkpoint, discard only the bad part ──
        Serial.print(F("ERROR in last input - reverted to last good state ("));
        Serial.print(g_lastGoodLen); Serial.println(F(" chars kept)."));
        if (g_lastGoodLen > 0) {
          Serial.print(F("Current buffer: ")); Serial.println(g_userBuf);
          Serial.println(F("Continue adding steps, or '!' to start over."));
        } else {
          Serial.println(F("Nothing saved yet - retype from scratch, e.g.: A1N E1T150 B2T350"));
        }
        trimToLastGood();
      }

    } else {
      // Normal character: just append
      if (g_userLen >= INPUT_MAX - 1) {
        Serial.println(F("Buffer full! Press Enter to save or '!' to clear."));
        return;
      }
      g_userBuf[g_userLen++] = c;
      g_userBuf[g_userLen]   = '\0';
    }
  }
}

// SETUP
void setup() {
  Serial.begin(9600);

  leftMotor.attach(LEFT,  500, 2500);
  leftMotor.setDirection(true);
  rightMotor.attach(RIGHT, 500, 2500);
  rightMotor.setDirection(false);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  stopMotors();

  Serial.println(F("==========================================="));
  Serial.println(F("  ROBOT DANCE "));
  Serial.println(F("  Send choreography via Serial (9600 baud)"));
  Serial.println(F("  then press BUTTON to start."));
  Serial.println(F("  Button during/after dance: return to base"));
  Serial.println(F("==========================================="));

  userBufReset();

  if (loadFromEEPROM()) {                                        // Try EEPROM first, then default
    ParseStatus ps = parseChoreography(g_choreoStr);
    if (ps == PS_OK) {
      Serial.print(F("Restored from EEPROM - steps=")); Serial.println(g_stepCount);
    } else {
      Serial.println(F("EEPROM data corrupt - loading default."));
      loadDefaultChoreography();
    }
  } else {
    Serial.println(F("No EEPROM data - loading default choreography."));
    loadDefaultChoreography();
  }

  Serial.print(F("Active: ")); Serial.println(g_choreoStr);
  Serial.println(F("Type a new choreography to override, then press BUTTON."));

  g_state = ST_IDLE;                                              // initial robot state
}

// MAIN LOOP
void loop() {
  if (g_state == ST_IDLE || g_state == ST_DANCE_DONE) {           // allow serial input only when robot is not moving
    readSerialInput();
  }

  int s0, s1, s2, s3, s4;
  readSensors(s0, s1, s2, s3, s4);
  bool inter = isIntersectionDebounced(s0, s1, s2, s3, s4);       // stable intersection signal
  static bool prevInter = false;

  if (buttonPressedEdge()) {                        // Button handling
    if (g_state == ST_IDLE) {
      startDance();
    } else if (g_state == ST_DANCE_DONE) {
      Serial.println(F("Returning to start..."));   // guarantees the return logic starts clean every time.
      g_retPhase = RET_INIT;
      g_state = ST_RETURN;
    } else {
      g_returnRequested = true;                     // set flag but do NOT force ST_RETURN immediately
      g_retPhase = RET_INIT;
      Serial.println(F("Return flagged - finishing current segment then returning to start."));
    }
  }

  switch (g_state) {                                // State machine
    case ST_IDLE:                                   // stationary state
    case ST_DANCE_DONE:
      stopMotors();
      digitalWrite(LED, LOW);
      break;

    case ST_FOLLOW: {                               // line followig + intersection detection
      if (g_needClearFirst) {
        if (!inter) {                               // right after leaving node, dont accept another node
          g_clearStreak++;                          // wait untill intersection is clearly gone for few cycles
          if (g_clearStreak >= CLEAR_STREAK_TH) {
            g_needClearFirst = false;
            g_clearStreak = 0;
          }
        } else {
          g_clearStreak = 0;
        }
      }

      bool enterEdge = (!prevInter && inter);       // detected entry

      if (!g_needClearFirst && millis() >= g_ignoreInterUntil && enterEdge) {
        advancePose();                             // update grid position
        Serial.print(F("Node: ")); printPose(); Serial.println();
        g_needClearFirst = true;
        g_clearStreak    = 0;

        if (g_returnRequested) {                   // in return mode: commit to intersection centre then hand back to doReturnToStart
          g_state = ST_INTER_COMMIT;
        } else {                                   // normal dance mode
          calculatePath(g_targetX, g_targetY, g_targetRowFirst);
          g_pendingTurn = (g_turnAmount != 0);
          g_state = ST_INTER_COMMIT;               // move into center
        }
      } else if (inter) {
        leftMotor.go(20); rightMotor.go(20);       // if over intersection but not counting it
      } else {
        followLine(s1, s2, s3);                    // normal line following
      }
      break;
    }

    case ST_INTER_COMMIT:                          // move a bit forward into intersection center
      doIntersectionCommit();
      break;

    case ST_AT_NODE:                               // wait for the absolute time and load next step or if return requested, go back
      handleAtNode();
      break;

    case ST_TURNING:                               // turns, update direction
      executeTurn(s0, s1, s2, s3, s4);
      break;
 
    case ST_ACQUIRE_FWD:                           // after turning, go but forward, reaquire line
      acquireForward(inter, s1, s2, s3);
      break;

    case ST_LEAVE_INTER:                           // make sure fully exit intersection before following again
      doLeaveIntersection(inter, s1, s2, s3);
      break;

    case ST_RETURN:                                // return function
      doReturnToStart();
      break;
  }
  prevInter = inter;                               // store current intersection state for next loop
  delay(2);                                        // slow loops slightly
}
