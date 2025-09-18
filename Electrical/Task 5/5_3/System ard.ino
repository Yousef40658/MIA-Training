////
//The code was fully solo written by me in the input phase and motors 
//used AI to help with tayloring the A* alogerithm,rewriting and adding comments
//i usually add comments by my own self but its 10:36 am and didn't sleep yet 
////



#include <Keypad.h>                 // Library to handle keypad input
#include <LiquidCrystal_I2C.h>      // Library to handle I2C LCD
#include <string.h>                 // For memset()

//////////////
// TARGET INPUT
//////////////

LiquidCrystal_I2C lcd(0x3F, 20, 4); // LCD component, address 0x3F, size 20x4

const byte ROWS = 4;
const byte COLS = 3;

// Keypad layout mapping
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Pin connections from Arduino to keypad rows and columns
byte rowPins[ROWS] = {10, 11, 12, 13}; // R1–R4
byte colPins[COLS] = {9, 8, 7};        // C1–C3

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// TARGET
String input = "";                // Stores the user input as string
int targetX = -1;                 // Final target X coordinate
int targetY = -1;                 // Final target Y coordinate
bool gotX = false;                // Flag for X coordinate received
bool gotY = false;                // Flag for Y coordinate received

// OBSTACLES
int obstacleCount = 0;            // Number of obstacles entered
bool got_n_obstacle = false;      // Flag to know if count received
#define MAXIMUM_OBSTACLES 5      // Limit on obstacles for safety

// Struct to store obstacle data
typedef struct obstacle {
  int x_cord;
  int y_cord;
  int width;
  int height;
  bool gotX_obst;
  bool gotY_obst;
  bool gotH_obst;
  bool gotW_obst;
} obstacle;

bool got_obstacles = false;       // Flag to check if memory allocation succeeded
obstacle *obstacles = NULL;       // Pointer to obstacle array

// GRID AND PATHFINDING
#define GRID_SIZE 5               // Size of the square grid (5x5)

struct Node {
  bool visited;
  bool inOpenSet;
  int gCost;                      // Cost from start to current
  int hCost;                      // Estimated cost to target
  int parentX;
  int parentY;
};

Node grid[GRID_SIZE][GRID_SIZE];  // Grid of nodes
bool isObstacle[GRID_SIZE][GRID_SIZE]; // Marks obstacle cells
int pathX[100];                   // X coordinates of found path
int pathY[100];                   // Y coordinates of found path
int pathLength = 0;
bool pathFound = false;
bool simulationDone = false;

// BONUS: Simulated wheel encoders
int encoderCount = 0;
unsigned long startTime;

//////////////////////////
// SETUP FUNCTION
//////////////////////////
void setup() {
  Serial.begin(9600);             // Start serial communication
  lcd.init();                     // Initialize LCD
  lcd.backlight();                // Turn on LCD backlight

  // Initial instructions on LCD
  lcd.setCursor(0, 0);
  lcd.print("#=Enter *=Clear");
  delay(1000);
  lcd.clear();
  lcd.print("Enter X : ");

  // Wait until both X and Y of the target are received
  while (!gotX || !gotY) {
    keypad_input_target();
  }

  // Ask for number of obstacles
  get_number_of_obstacles();

  // Limit number of obstacles
  if (obstacleCount > MAXIMUM_OBSTACLES) {
    obstacleCount = MAXIMUM_OBSTACLES;
  }

  // Allocate memory for obstacles
  obstacles = (obstacle*)calloc(obstacleCount, sizeof(obstacle));
  if (obstacles != NULL) {
    got_obstacles = true;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Alloc Failed!");
    while (1); // Halt system
  }

  // Prompt to input obstacle details
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Input Obstacles");
  delay(500);

  // Loop through and receive specs of each obstacle
  for (int i = 0; i < obstacleCount; i++) {
    obstacles[i].gotX_obst = false;
    obstacles[i].gotY_obst = false;
    obstacles[i].gotH_obst = false;
    obstacles[i].gotW_obst = false;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Obstacle " + String(i + 1));
    get_spec(&(obstacles[i].gotX_obst), &(obstacles[i].x_cord), "X cord");
    get_spec(&(obstacles[i].gotY_obst), &(obstacles[i].y_cord), "Y cord");
    get_spec(&(obstacles[i].gotH_obst), &(obstacles[i].height), "Height");
    get_spec(&(obstacles[i].gotW_obst), &(obstacles[i].width), "Width");
  }

  // Mark obstacles on the grid
  memset(isObstacle, 0, sizeof(isObstacle)); // Clear grid
  for (int i = 0; i < obstacleCount; i++) {
    int x0 = obstacles[i].x_cord;
    int y0 = obstacles[i].y_cord;
    int w = obstacles[i].width;
    int h = obstacles[i].height;
    for (int x = x0; x < x0 + w && x < GRID_SIZE; x++) {
      if (x >= 0) {
        for (int y = y0; y < y0 + h && y < GRID_SIZE; y++) {
          if (y >= 0) {
            isObstacle[x][y] = true; // Mark cell as obstacle
          }
        }
      }
    }
  }

  // Final setup confirmation
  lcd.clear();
  lcd.setCursor(0, 0);
  delay(500);
}

void loop() {
  if (!pathFound) {
    // If path not found yet, attempt A* pathfinding
    if (findPath()) {
      pathFound = true;               // Mark path as found
      startTime = millis();           // Start timing for encoder simulation
    } else {
      Serial.println("No path found");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("No path found");

      while (1);                      // Stop execution if no path is found
    }
  }

  else if (!simulationDone) {
    // Simulate robot movement through calculated path
    for (int i = 0; i < pathLength; i++) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Robot at:");
      lcd.setCursor(0, 1);
      lcd.print("(" + String(pathX[i]) + "," + String(pathY[i]) + ")");

      encoderCount += 100;           // Simulate 100 ticks per grid move

      unsigned long currentTime = millis();
      float timeElapsed = (currentTime - startTime) / 1000.0;  // in seconds
      float distance = i + 1;        // each step counts as 1 unit
      float speed = (timeElapsed > 0) ? distance / timeElapsed : 0; // units/sec

      // Print robot position and speed info to serial
      Serial.print("Robot at (");
      Serial.print(pathX[i]);
      Serial.print(",");
      Serial.print(pathY[i]);
      Serial.print(") | Encoder: ");
      Serial.print(encoderCount);
      Serial.print(" | Speed: ");
      Serial.print(speed);

      // 🧠 Determine movement direction from previous step
      String direction = "FWD";      // Default movement is forward

      if (i > 0) {
        int dx = pathX[i] - pathX[i - 1];
        int dy = pathY[i] - pathY[i - 1];

        if (dx == 1)        direction = "RIGHT";
        else if (dx == -1)  direction = "LEFT";
        else if (dy == 1)   direction = "FWD";   // Up
        else if (dy == -1)  direction = "BWD";   // Down
      }

      // Send direction and speed to motor Arduino
      // will do it seperatly as no pins are left in #1
      int pwmSpeed = speed * 100;                // Scale speed to PWM
      pwmSpeed = constrain(pwmSpeed, 0, 255);    // Clamp to valid range

      Serial.print('<');
      Serial.print("SPD:");
      Serial.print(pwmSpeed);
      Serial.print(',');
      Serial.print(direction);
      Serial.println('>');  // Format: "<SPD:255,FWD>"

      delay(1000); // Simulate delay for each step
    }

    // Path completed
    Serial.println("Reached target!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reached target!");

    Serial.print('<');
    Serial.print("SPD:0,STOP");
    Serial.println('>');   // Send stop command

    simulationDone = true; // Mark simulation as done
  }
}


//////////////////////
// GET TARGET (X,Y)
//////////////////////
void keypad_input_target() {
  char key = keypad.getKey();      // Read key from keypad

  if (key) {
    if (key >= '0' && key <= '9') {
      input += key;                // Add digit to input string
      lcd.setCursor(0, 1);
      lcd.print(input + "            ");    // Show input on LCD
    }

    else if (key == '#') {         // Confirm input
      if (!gotX) {
        targetX = input.toInt();   // Convert string to int for X
        input = "";
        gotX = true;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter Y : ");
      }

      else if (!gotY) {
        targetY = input.toInt();   // Convert string to int for Y
        input = "";
        gotY = true;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Target:");
        lcd.setCursor(0, 1);
        lcd.print("(" + String(targetX) + "," + String(targetY) + ")");
        delay(2000);
        lcd.clear();
      }
    }

    else if (key == '*') {         // Clear input
      input = "";
      lcd.setCursor(0, 1);
      lcd.print("    ");           // Erase input display
    }
  }
}

////////////////////////////
// GET OBSTACLES
////////////////////////////
void get_number_of_obstacles() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter # obstacles");   // Instruction to user

  lcd.setCursor(0, 1);
  lcd.print("Input: ");
  input = "";

  while (!got_n_obstacle) {
    char key = keypad.getKey();     // Wait for keypad input

    if (key) {
      if (key >= '0' && key <= '9') {
        input += key;               // Add digit to input string
        lcd.setCursor(7, 1);        // Position after "Input: "
        lcd.print(input + "   ");   // Show updated input
      }

      else if (key == '#') {        // Confirm
        obstacleCount = input.toInt();
        input = "";
        got_n_obstacle = true;
      }

      else if (key == '*') {        // Clear
        input = "";
        lcd.setCursor(7, 1);
        lcd.print("    ");          // Erase input
      }
    }

    delay(50);                      // Debounce delay
  }
}


void get_spec(bool* flag, int* spec, const char* spec_name) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter " + String(spec_name));  // Show which value is needed

  lcd.setCursor(0, 1);
  lcd.print("Input: ");
  input = "";

  while (!*flag) {
    char key = keypad.getKey();     // Wait for key input

    if (key) {
      if (key >= '0' && key <= '9') {
        input += key;
        lcd.setCursor(7, 1);        // After "Input: "
        lcd.print(input + "   ");   // Show value
      }

      else if (key == '#') {        // Confirm
        *spec = input.toInt();
        input = "";
        *flag = true;
      }

      else if (key == '*') {        // Clear input
        input = "";
        lcd.setCursor(7, 1);
        lcd.print("    ");          // Erase
      }
    }

    delay(50);                      // Debounce
  }
}

///////////////////////
// PATH FINDING using A*
// https://www.geeksforgeeks.org/dsa/a-search-algorithm/
// read the article but run into multiple problems -mostly memory-
// so i had Ai tailor it for the task but i got its idea is like the minimax of
///////////////////////
bool findPath() {
  // Return false if start or goal is inside obstacle
  if (isObstacle[0][0] || isObstacle[targetX][targetY]) {
    return false;
  }

  // Initialize grid nodes
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      grid[x][y].visited = false;
      grid[x][y].inOpenSet = false;
      grid[x][y].gCost = 9999;                                  // Large initial cost
      grid[x][y].hCost = abs(x - targetX) + abs(y - targetY);   // Manhattan distance
      grid[x][y].parentX = -1;
      grid[x][y].parentY = -1;
    }
  }

  grid[0][0].gCost = 0;              // Starting node cost
  grid[0][0].inOpenSet = true;       // Add to open set

  while (true) {
    int minFCost = 999999;
    int currentX = -1, currentY = -1;

    // Find node in open set with lowest fCost = gCost + hCost
    for (int x = 0; x < GRID_SIZE; x++) {
      for (int y = 0; y < GRID_SIZE; y++) {
        if (grid[x][y].inOpenSet && !grid[x][y].visited) {
          int fCost = grid[x][y].gCost + grid[x][y].hCost;
          if (fCost < minFCost) {
            minFCost = fCost;
            currentX = x;
            currentY = y;
          }
        }
      }
    }

    if (currentX == -1) {
      return false;  // No path found (open set empty)
    }

    // Goal reached
    if (currentX == targetX && currentY == targetY) {
      // Reconstruct path by tracing parents
      int pathTempX[100];
      int pathTempY[100];
      int pathIdx = 0;

      int cx = currentX, cy = currentY;
      while (cx != -1 && cy != -1) {
        pathTempX[pathIdx] = cx;
        pathTempY[pathIdx] = cy;
        pathIdx++;

        int px = grid[cx][cy].parentX;
        int py = grid[cx][cy].parentY;
        cx = px;
        cy = py;
      }

      // Reverse path to get correct order (start → target)
      pathLength = pathIdx;
      for (int i = 0; i < pathLength; i++) {
        pathX[i] = pathTempX[pathLength - 1 - i];
        pathY[i] = pathTempY[pathLength - 1 - i];
      }

      return true;
    }

    // Mark current node as visited
    grid[currentX][currentY].visited = true;
    grid[currentX][currentY].inOpenSet = false;

    // Check all 4 neighbors (up, down, left, right)
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++) {
      int nx = currentX + dx[i];
      int ny = currentY + dy[i];

      // Bounds check + skip visited + skip obstacles
      if (nx >= 0 && nx < GRID_SIZE &&
          ny >= 0 && ny < GRID_SIZE &&
          !isObstacle[nx][ny] &&
          !grid[nx][ny].visited) {

        int tentative_gCost = grid[currentX][currentY].gCost + 1;

        // Better path found?
        if (tentative_gCost < grid[nx][ny].gCost) {
          grid[nx][ny].gCost = tentative_gCost;
          grid[nx][ny].parentX = currentX;
          grid[nx][ny].parentY = currentY;

          // Add to open set if not already there
          if (!grid[nx][ny].inOpenSet) {
            grid[nx][ny].inOpenSet = true;
          }
        }
      }
    }
  }
}
