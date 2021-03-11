#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>

#define TIME_STEP 32
#define MAX_SPEED 10

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//Creating new robot instance
Robot* robot = new Robot();

//Get devices
Motor* leftMotor = robot->getMotor("Wheel_L");
Motor* rightMotor = robot->getMotor("Wheel_R");
Motor* leftArmMotor = robot->getMotor("Arm_L");
Motor* rightArmMotor = robot->getMotor("Arm_R");
GPS* gps = robot->getGPS("gps");
Compass* compass = robot->getCompass("compass");
LightSensor* lightSensor = robot->getLightSensor("TEPT4400");
DistanceSensor* ds[3];
char dsNames[3][20] = { "us_right","us_left","ir" };

//Initiate vector to store sensor readings
std::vector<std::vector<double>> sensorValueScan;


/*==================================================
MOTION FUNCTIONS
===================================================*/

void move_forwards() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(0.5 * MAX_SPEED);
}

void move_backwards() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(-0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);
}

void rotate_CW() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);
}

void rotate_ACW() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(-0.5 * MAX_SPEED);
    rightMotor->setVelocity(0.5 * MAX_SPEED);
}

void open_arms() {
    leftArmMotor->setPosition(1);
    rightArmMotor->setPosition(-1);
}

void close_arms() {
    Motor* leftArmMotor = robot->getMotor("Arm_L");
    Motor* rightArmMotor = robot->getMotor("Arm_R");
    leftArmMotor->setPosition(0.3);
    rightArmMotor->setPosition(-0.3);

}

void shuffleBack() {

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(-0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);

    int i =0;
    
    while(robot->step(TIME_STEP) != -1){
      i++;
      if(i==70){
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        std::cout << "shuffleBack success" << std::endl;
        break;
      }
      
    }
}

/*===========================================================================
GET SENSOR AND BEARING FUNCTIONS
=============================================================================*/

//Function to get sensor values for one time step, in distance units
std::vector<double> getSensorValues(double (*get_bearing_in_degrees)()) {

    //Creating vector to store one row of sensor values for one time step
    std::vector<double> sensorValues(4);

    //Input distance sensor values as distance (involves dividing through by 57)
    for (int i = 0; i < 2; i++) {
        sensorValues[i] = (ds[i]->getValue()) / 5700;
    }

    //Get IR sensor lookup table
    const double* lookUpTable = ds[2]->getLookupTable();
    int lookUpTableSize = ds[2]->getLookupTableSize();


    //Input IR sensor values as distance (involves interpolation)

    double distanceActual = 0;
    double lookUpValue = ds[2]->getValue();
    for (int j = 0; j < lookUpTableSize; j++) {
        double voltageRef = lookUpTable[3 * j + 1];
        double distanceRef = lookUpTable[3 * j];
        if (lookUpValue > voltageRef) {
            distanceActual = lookUpTable[3 * (j - 1)] + (distanceRef - lookUpTable[3 * (j - 1)]) * (lookUpValue - voltageRef);
            break;
        }
    }
    sensorValues[2] = distanceActual;

    //Last value of size 5 vector is bearing
    double bearing = get_bearing_in_degrees();
    sensorValues[3] = bearing;
    return sensorValues;
}

std::vector<double> getRawSensorValues(double (*get_bearing_in_degrees)()) {

    //Creating vector to store one row of sensor values for one time step
    std::vector<double> sensorValues(4);

    //Input distance sensor values as distance (involves dividing through by 57)
    for (int i = 0; i < 3; i++) {
        sensorValues[i] = ds[i]->getValue();
    }
    return sensorValues;
}

double get_bearing_in_degrees() {
    const double* north = compass->getValues();
    double rad = atan2(north[0], north[2]);
    double bearing = rad / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
    }
    return bearing;
}

/*======================================================================
SCANNING FUNCTIONS
========================================================================*/

//Function to rotate by a fixed angle
void rotate_theta(double theta, double initial_bearing, bool& fin) {

    double angle_rotated = 0.0;
    // set the target position, velocity of the motors
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);

    while (robot->step(TIME_STEP) != -1) {
        double bearing = get_bearing_in_degrees();
        if ((bearing - initial_bearing) >= 0.0) {
            angle_rotated = bearing - initial_bearing;
        }

        if ((bearing - initial_bearing) < 0.0) {
            angle_rotated = bearing + (360.0 - initial_bearing);
        }
        //std::cout << angle_rotated << std::endl;
        //std::cout << theta << std::endl;

        if (angle_rotated > theta) {
            leftMotor->setVelocity(0.0 * MAX_SPEED);
            rightMotor->setVelocity(0.0 * MAX_SPEED);
            fin = true;
            break;
        }
    }
}

//Version of rotate_theta that also scans while doing so
void doScan(double theta, double initial_bearing, bool* fin) {

    double angle_rotated = 0.0;
    // set the target position, velocity of the motors
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.3 * MAX_SPEED);
    rightMotor->setVelocity(-0.3 * MAX_SPEED);

    while (robot->step(TIME_STEP) != -1) {
        double bearing = get_bearing_in_degrees();
        std::vector<double> Values = getSensorValues(get_bearing_in_degrees);
        sensorValueScan.push_back(Values);
        if ((bearing - initial_bearing) >= 0) {
            angle_rotated = bearing - initial_bearing;
        }

        if ((bearing - initial_bearing) < 0) {
            angle_rotated = bearing + (360 - initial_bearing);
        }
        //std::cout << angle_rotated << std::endl;
        //std::cout << theta << std::endl;

        if (angle_rotated > theta) {
            leftMotor->setVelocity(0);
            rightMotor->setVelocity(0);
            *fin = true;
            break;
        }
    }
}

void rotate_until_bearing(double target_bearing, double initial_bearing) {

    double angle_rotated = 0.0;
    // set the target position, velocity of the motors
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    if (target_bearing > initial_bearing) {

        leftMotor->setVelocity(0.5 * MAX_SPEED);
        rightMotor->setVelocity(-0.5 * MAX_SPEED);

        while (robot->step(TIME_STEP) != -1) {

            double bearing = get_bearing_in_degrees();

            if (bearing >= target_bearing) {

                leftMotor->setVelocity(0.0 * MAX_SPEED);
                rightMotor->setVelocity(0.0 * MAX_SPEED);

                break;

            }
        }
    }

    else {

        leftMotor->setVelocity(-0.1 * MAX_SPEED);
        rightMotor->setVelocity(0.1 * MAX_SPEED);

        while (robot->step(TIME_STEP) != -1) {
            double bearing = get_bearing_in_degrees();
            //std::cout << bearing << std::endl;

            if (bearing <= target_bearing) {
                leftMotor->setVelocity(0.0 * MAX_SPEED);
                rightMotor->setVelocity(0.0 * MAX_SPEED);
                break;

            }
        }
    }
}

/*================================================================
BLOCK DETECTION
==================================================================*/

//Takes uncleaned 2d vector and filters for blocks, returning array of their bearings
std::vector<double> getBlockBearings() {

    std::vector<double> blockBearings(8);

    //Calculating average returned IR sensor value
    int lookUpTableSize = ds[2]->getLookupTableSize();
    int numCounter = 0;
    double sumDistance = 0;
    for (unsigned int i = 0; i < sensorValueScan.size(); i++) {
        sumDistance += sensorValueScan[i][2];
        if (sensorValueScan[i][2] != 0) {
            numCounter++;
        }
    }

    double avgDistance = sumDistance / numCounter;

    //Alpha is a shortcode for the distance value on the jth row, 2nd column of sensorValueScam
    double alpha;
    int j = 0;
    //Conditional logic to extract bearings and then convert to GPS location
    for (unsigned int i = 1; i < sensorValueScan.size(); i++) {
        alpha = sensorValueScan[i][2];
        /*Conditions for blocks to be picked out:
        1. Large jump between previous value
        2. Large difference between distance value recorded and average distance value
        calculated above
        */
        if ((sensorValueScan[i - 1][2] - alpha) > 0.05) {
            blockBearings[j] = sensorValueScan[i][3];
            j++;
        }
    }

    return blockBearings;
}

std::vector<double> getBlockDistances() {

    std::vector<double> blockDistances(8);

    //Calculating average returned IR sensor value
    int lookUpTableSize = ds[2]->getLookupTableSize();
    int numCounter = 0;
    double sumDistance = 0;
    for (unsigned int i = 0; i < sensorValueScan.size(); i++) {
        sumDistance += sensorValueScan[i][2];
        if (sensorValueScan[i][2] != 0) {
            numCounter++;
        }
    }

    double avgDistance = sumDistance / numCounter;

    //Alpha is a shortcode for the distance value on the jth row, 2nd column of sensorValueScam
    double alpha;
    //Conditional logic to extract bearings and then convert to GPS location
    for (unsigned int i = 1; i < sensorValueScan.size(); i++) {
        alpha = sensorValueScan[i][2];
        /*Conditions for blocks to be picked out:
        1. Large jump between previous value
        2. Large difference between distance value recorded and average distance value
        calculated above
        */
        if ((sensorValueScan[i - 1][2] - alpha) > 0.05) {
            blockDistances[i] = alpha;
        }
    }

    return blockDistances;
}
//Clean sensorValueScan to get block GPS coordinates. A copy of getBlockBearings with GPS added
std::vector<std::vector<double>> getBlockGPS() {

    std::vector<double> blockBearings(8);
    std::vector<double> blockDistances(8);

    //Calculating average returned IR sensor value
    int lookUpTableSize = ds[2]->getLookupTableSize();
    int numCounter = 0;
    double sumDistance = 0;
    for (unsigned int i = 0; i < sensorValueScan.size(); i++) {
        sumDistance += sensorValueScan[i][2];
        if (sensorValueScan[i][2] != 0) {
            numCounter++;
        }
    }

    double avgDistance = sumDistance / numCounter;
    std::vector<std::vector<double>> blockGPS(8, std::vector<double>(3));

    //Alpha is a shortcode for the distance value on the jth row, 2nd column of sensorValueScam
    double alpha;
    int j = 0;
    //Conditional logic to extract bearings and then convert to GPS location
    for (unsigned int i = 1; i < sensorValueScan.size(); i++) {
        alpha = sensorValueScan[i][2];
        /*Conditions for blocks to be picked out:
        1. Large jump between previous value
        2. Large difference between distance value recorded and average distance value
        calculated above
        */
        if ((sensorValueScan[i - 1][2] - alpha) > 0.05) {
            blockBearings[j] = sensorValueScan[i][3];
            blockDistances[j] = alpha;
            j++;

        }

    }

    //Turning bearings and distances into new GPS readings
    for (int i = 0; i < 8; i++) {
        blockGPS[i][0] = gps->getValues()[0] + (blockDistances[i] + 0.12) * cos(blockBearings[i] * M_PI / 180);
        blockGPS[i][1] = gps->getValues()[1];
        blockGPS[i][2] = gps->getValues()[2] + (blockDistances[i] + 0.12) * sin(blockBearings[i] * M_PI / 180);
    }

    return blockGPS;
}

/*=================================================================
GET COLOUR
==================================================================*/
bool getColour() {
    double LSVoltage = lightSensor->getValue();
    if (LSVoltage > 0.65) {
        return 1;
    }
    else {
        return 0;
    }

}
/*================================================================
RETURN TO START
==================================================================*/
void return_to_initial_position() {

    double initial_position[3]={0,0,-0.4};
    double current_position[3];
    current_position[0] = gps->getValues()[0];
    current_position[1] = gps->getValues()[1];
    current_position[2] = gps->getValues()[2];

    double target_bearing = 0.0;

    if (current_position[2] < initial_position[2]) {
        target_bearing = 90.0 + (std::atan((current_position[0] - initial_position[0]) / (current_position[2] - initial_position[2])) * 180.0 / M_PI);
        std::cout << "condition 1" << std::endl;
    }

    if (current_position[2] > initial_position[2]) {
        target_bearing = 270.0 - (std::atan((current_position[0] - initial_position[0]) / (current_position[2] - initial_position[2])) * 180.0 / M_PI);
        std::cout << "condition 2" << std::endl;
    }

    if (current_position[2] == initial_position[2] && current_position[0] > initial_position[0]) {
        target_bearing = 180.0;
        std::cout << "condition 3" << std::endl;
    }

    if (current_position[2] == initial_position[2] && current_position[0] < initial_position[0]) {
        target_bearing = 0.0;
        std::cout << "condition 4" << std::endl;
    }

    if (current_position[2] == initial_position[2] && current_position[0] == initial_position[0]) {
        // need to exit function
        std::cout << "condition 5" << std::endl;
    }

    double distance_to_travel = sqrt(pow((current_position[0] - initial_position[0]), 2) 
    + pow((current_position[2] - initial_position[2]), 2));
    //double wheel_angle_to_rotate = distance_to_travel / 0.02;
    double initial_bearing = get_bearing_in_degrees(); 

    rotate_until_bearing(target_bearing, initial_bearing);

    move_forwards();

    while (robot->step(TIME_STEP) != -1) {

          double distance = sqrt(pow(gps->getValues()[0], 2)
              + pow(-0.4 - gps->getValues()[2], 2));

          if (distance <= 0.2) {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              break;
          }

          if (robot->step(TIME_STEP) == 1) {
              break;
          }

      }

}
/*================================================================
MAIN FUNCTION
==================================================================*/

int main(int argc, char** argv) {

    //Enabling devices
    gps->enable(TIME_STEP);
    compass->enable(TIME_STEP);
    lightSensor->enable(TIME_STEP);
    for (int i = 0; i < 3; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
    }

    int i = 0;

    bool scanblocks = false;
    bool gotblock = false;
    bool moveblock = false;

    //MAIN WHILE LOOP WHERE STUFF HAPPENS
    while (robot->step(TIME_STEP) != -1) {

        //Code snippet to get sensor values as bot rotates in circle
        /*std::vector<double> Values = getRawSensorValues(get_bearing_in_degrees);
        sensorValueScan.push_back(Values);
        rotate_ACW();*/

        //Initial scan
        if (scanblocks == false) {
            double currentBearing = get_bearing_in_degrees();
            doScan(355, currentBearing, &scanblocks);
            scanblocks = true;
        }

        /*if(robot->step(TIME_STEP) == -1){
          break;
         }*/

         /*if(fin1==true && fin2==false){
            std::cout << "Debug if 2" << std::endl;
            std::vector<double> blockBearings = getBlockBearings();
            for(auto& i:blockBearings){
                std::cout << i << std::endl;
               }
            std::vector<std::vector<double>> GPSOfBlocks = getBlockGPS(&fin2);
            for(auto& row:GPSOfBlocks){
              for(auto& i:row){
                 std::cout << i << std::endl;
              }
              std::cout << "NEXT" << std::endl;
            }
            fin2 = true;
         }*/
         //Collecting one block
        if (gotblock == false) {
            
            if(i==1){
              std::cout << "Attempt at second block" << std::endl;
             }
             
            std::vector<double> bearings = getBlockBearings();
            std::vector<std::vector<double>> GPSOfBlocks = getBlockGPS();
            std::cout << "Issue not with calling bearings and GPS again" << std::endl;
            rotate_until_bearing(bearings[i], get_bearing_in_degrees());
            std::cout << "Finished rotating" << std::endl;

            open_arms();
            move_forwards();

            while (robot->step(TIME_STEP) != -1) {

                double distance = sqrt(pow(GPSOfBlocks[i][0] - gps->getValues()[0], 2)
                    + pow(GPSOfBlocks[i][2] - gps->getValues()[2], 2));

                if (distance < 0.07) {
                    leftMotor->setVelocity(0.0);
                    rightMotor->setVelocity(0.0);
                    bool colour = getColour();

                    if (colour == 1) {  
                        std::cout << "Red block found" << std::endl;
                        shuffleBack();
                        break;
                    }
                    else {
                        close_arms();
                        std::cout << "Green block found" << std::endl;
                    }
                    moveblock=false;
                    break;

                }

                if (robot->step(TIME_STEP) == 1) {
                    break;
                }

            }

            gotblock = true;
        }
        
        if (moveblock == false) {

           return_to_initial_position();
           open_arms();
           std::cout << "Problem occurs after opening arms" << std::endl;
           shuffleBack();
           moveblock = true;
           gotblock = false;
        }

        i++;

        if (i == 8) {
            break;
        }

        /*=========================================
        CODE SNIPPET TO OUTPUT SENSOR VALUE 2D ARRAY
        ===========================================*/

        /*//Outputting sensorValueScan
        for(int j = 0; j<4; j++){
          std::cout << sensorValueScan[i][j]<< ",";
        }
        std::cout << "\n";*/

        /*============================================
        CODE SNIPPET TO OUTPUT AVERAGE DISTANCE
        ============================================*/

        //Output average distance
        /*int lookUpTableSize = ds[2]->getLookupTableSize();
          if(i==30){
          int numCounter = 0;
          double sumDistance = 0;
          for(int j = 0; j<lookUpTableSize;j++){
            sumDistance += sensorValueScan[j][2];
            if(sensorValueScan[j][2]!=0){
              numCounter++;
            }
          }
          double avgDistance = sumDistance/numCounter;
          std::cout << "DISTANCE AVERAGE:" << avgDistance;
        }*/

        /*=============================================================
        CODE SNIPPET FOR OUTPUTTING LOOKUP TABLE VALUES
        ===============================================================
        */
        /*if(i==0){
          const double *lookUpTable = ds[2]->getLookupTable();
          int lookUpTableSize = ds[2]->getLookupTableSize();
          for(int j=0; j<lookUpTableSize;j++){
            for(int k=0; k<3;k++){
              std::cout << lookUpTable[3*j+k] << ",";
            }
            std::cout << "\n";
           }
          i++;
        }*/

    } //bracket to end while loop

    delete robot;
    return 0;
}