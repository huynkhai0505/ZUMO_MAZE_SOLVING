#include <Wire.h>
#include <ZumoShield.h>

#define ZUMO_SENSOR_THRESHOLD 300

#define DETECTS_LINE(sensor)((sensor) > ZUMO_SENSOR_THRESHOLD)
#define ZUMO_TURNING_SPEED 200

#define SPEED 200
#define LINE_THICKNESS_ALLOW .75

#define INCHES_TO_ZUNITS 17142.0

#define OVERSHOOT(LINE_THICKNESS_ALLOW)(((INCHES_TO_ZUNITS * (LINE_THICKNESS_ALLOW)) / SPEED))

ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

// path[] keeps a log of all the turns made
// since starting the maze
char path[100] = "";
unsigned char path_length = 0; // the length of the path

void setup()
{

  unsigned int sensors[6];
  unsigned short count = 0;
  unsigned short last_status = 0;
  int turn_direction = 1;

  buzzer.play(">g32>>c32");

  reflectanceSensors.init();

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);        // turn on LED to indicate we are in calibration mode

  button.waitForButton();

  // Calibrate the Zumo by sweeping it from left to right
  for(int i = 0; i < 4; i ++)
  {
    turn_direction *= -1;
    motors.setSpeeds(turn_direction * ZUMO_TURNING_SPEED, -1*turn_direction * ZUMO_TURNING_SPEED);

    while(count < 2)
    {
      reflectanceSensors.calibrate();
      reflectanceSensors.readLine(sensors);
      if(turn_direction < 0)
      {
        count += DETECTS_LINE(sensors[5]) ^ last_status;
        last_status = DETECTS_LINE(sensors[5]);
      }
      else
      {
        count += DETECTS_LINE(sensors[0]) ^ last_status;
        last_status = DETECTS_LINE(sensors[0]);
      }
    }

    count = 0;
    last_status = 0;
  }

  // Turn left.
  turn('L');

  motors.setSpeeds(0, 0);

  buzzer.play("L16 cdegreg4");
  digitalWrite(13, LOW);
}

void loop()
{
  solveMaze();
  buzzer.play(">>a32");
  while(1)
  {
    button.waitForButton();
    goToFinishLine();
    buzzer.play(">>a32");
  }
}
void turn(char dir)
{

  // count and last_status help
  // keep track of how much further
  // the Zumo needs to turn.
  unsigned short count = 0;
  unsigned short last_status = 0;
  unsigned int sensors[6];

  // dir tests for which direction to turn
  switch(dir)
  {
    case 'L':
  case 'B':
      motors.setSpeeds(-ZUMO_TURNING_SPEED, ZUMO_TURNING_SPEED);
      while(count < 2)
      {
        reflectanceSensors.readLine(sensors);
        count += DETECTS_LINE(sensors[1]) ^ last_status;
        last_status = DETECTS_LINE(sensors[1]);
      }

    break;

    case 'R':
      // Turn right.
      motors.setSpeeds(ZUMO_TURNING_SPEED, -ZUMO_TURNING_SPEED);
      while(count < 2)
      {
        reflectanceSensors.readLine(sensors);
        count += DETECTS_LINE(sensors[4]) ^ last_status;
        last_status = DETECTS_LINE(sensors[4]);
      }

    break;

    case 'S':
    break;
  }
}
char selectTurn(unsigned char found_left, unsigned char found_straight,
  unsigned char found_right)
{
  if(found_left)
    return 'L';
  else if(found_straight)
    return 'S';
  else if(found_right)
    return 'R';
  else
    return 'B';
}

void followSegment()
{
  unsigned int position;
  unsigned int sensors[6];
  int offset_from_center;
  int power_difference;

  while(1)
  {
    // Get the position of the line.
    position = reflectanceSensors.readLine(sensors);
    offset_from_center = ((int)position) - 2500;
    power_difference = offset_from_center / 3;

    // Compute the actual motor settings. 
    if(power_difference > SPEED)
      power_difference = SPEED;
    if(power_difference < -SPEED)
      power_difference = -SPEED;

    if(power_difference < 0)
      motors.setSpeeds(SPEED + power_difference, SPEED);
    else
      motors.setSpeeds(SPEED, SPEED - power_difference);
    if(!DETECTS_LINE(sensors[0]) && !DETECTS_LINE(sensors[1]) && !DETECTS_LINE(sensors[2]) && !DETECTS_LINE(sensors[3]) && !DETECTS_LINE(sensors[4]) && !DETECTS_LINE(sensors[5]))
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if(DETECTS_LINE(sensors[0]) || DETECTS_LINE(sensors[5]))
    {
      // Found an intersection.
      return;
    }

  }
}

void solveMaze()
{
    while(1)
    {
        // Navigate current line segment
        followSegment();
        unsigned char found_left = 0;
        unsigned char found_straight = 0;
        unsigned char found_right = 0;

        // Now read the sensors and check the intersection type.
        unsigned int sensors[6];
        reflectanceSensors.readLine(sensors);

        // Check for left and right exits.
        if(DETECTS_LINE(sensors[0]))
            found_left = 1;
        if(DETECTS_LINE(sensors[5]))
            found_right = 1;
        motors.setSpeeds(SPEED, SPEED);
        delay(OVERSHOOT(LINE_THICKNESS_ALLOW)/2);

        reflectanceSensors.readLine(sensors);

        // Check for left and right exits.
        if(DETECTS_LINE(sensors[0]))
            found_left = 1;
        if(DETECTS_LINE(sensors[5]))
            found_right = 1;
        delay(OVERSHOOT(LINE_THICKNESS_ALLOW)/2);

        // Check for a straight exit.
        reflectanceSensors.readLine(sensors);

        // Check again to see if left or right segment has been found
        if(DETECTS_LINE(sensors[0]))
            found_left = 1;
        if(DETECTS_LINE(sensors[5]))
            found_right = 1;

        if(DETECTS_LINE(sensors[1]) || DETECTS_LINE(sensors[2]) || DETECTS_LINE(sensors[3]) || DETECTS_LINE(sensors[4]))
            found_straight = 1;
        if(DETECTS_LINE(sensors[1]) && DETECTS_LINE(sensors[2]) && DETECTS_LINE(sensors[3]) && DETECTS_LINE(sensors[4]))
        {
          motors.setSpeeds(0,0);
          break;
        }

        // Intersection identification is complete.
        unsigned char dir = selectTurn(found_left, found_straight, found_right);

        // Make the turn indicated by the path.
    turn(dir);

        // Store the intersection in the path variable.
        path[path_length] = dir;
        path_length++;
        simplifyPath();

    }
}

// Now enter an infinite loop - we can re-run the maze as many
// times as we want to.
void goToFinishLine()
{
  unsigned int sensors[6];
  int i = 0;

  // Turn around if the Zumo is facing the wrong direction.
  if(path[0] == 'B')
  {
    turn('B');
    i++;
  }

  for(;i<path_length;i++)
  {

    followSegment();

    // Drive through the intersection.
    motors.setSpeeds(SPEED, SPEED);
    delay(OVERSHOOT(LINE_THICKNESS_ALLOW));

    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[i]);
  }

  // Follow the last segment up to the finish.
  followSegment();

  // The finish line has been reached.
  // Return and wait for another button push to
  // restart the maze.
  reflectanceSensors.readLine(sensors);
  motors.setSpeeds(0,0);

  return;
}

void simplifyPath()
{

  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length - 2] != 'B')
  return;

  int total_angle = 0;
  int i;

  for(i = 1; i <= 3; i++)
  {
    switch(path[path_length - i])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch(total_angle)
  {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}
