#ifndef CB_OBSTACLEAVOIDANCE_H_
#define CB_OBSTACLEAVOIDANCE_H_

#include <Arduino.h>

class CB_ObstacleAvoidance{
public:
    // Constructors
    CB_ObstacleAvoidance();
    CB_ObstacleAvoidance(int trigPin, int *echoPins, int n);
    
    // Destructors
    ~CB_ObstacleAvoidance();

    // Getters
    int getEchoPin(int index);
    int getEchoPinSize();
    int getLeftSensorValue();
    int getForwardSensorValue();
    int getRightSensorValue();
    int getAvoidDistance();

    // Setters
    void setEchoPin(int pin, int index);
    void setEchoPinSize(int n);
    void setLeftSensorPin(int pin);
    void setForwardSensorPin(int pin);
    void setRightSensorPin(int pin);
    void setAvoidDistance(int dist);

    // Utilities
    void initObstacleAvoidance();
    void printAllEchoPins();

    // Functionality
    int findDistance(int index);

private:
    int _trigPin;
    int *_echoPins = new int(0);
    int _numEchoPins;
    int _leftSensor, _forwardSensor, _leftSensor;
    int _avoidDistance;
};

#endif