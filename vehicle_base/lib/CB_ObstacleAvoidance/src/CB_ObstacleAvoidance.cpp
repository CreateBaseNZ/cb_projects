#include <CB_ObstacleAvoidance.h>

// ------------------------- Constructors ------------------------- //

CB_ObstacleAvoidance::CB_ObstacleAvoidance(){
    _trigPin = 2;
    *_echoPins = 3;
    *(_echoPins+1) = 4;
    *(_echoPins+2) = 5;
    _numEchoPins = 3;
}

CB_ObstacleAvoidance::CB_ObstacleAvoidance(int trigPin, int *echoPins, int n){
    _trigPin = trigPin;
    _echoPins = echoPins;
    _numEchoPins = n;
}

// ------------------------- Destructors ------------------------- //

CB_ObstacleAvoidance::~CB_ObstacleAvoidance(){
    delete _echoPins;
}

// ------------------------- Getters ------------------------- //



int CB_ObstacleAvoidance::getEchoPin(int index){
    return *(_echoPins + index);
}

int CB_ObstacleAvoidance::getEchoPinSize(){
    return _numEchoPins;
}

// ------------------------- Utilities ------------------------- //

void CB_ObstacleAvoidance::initObstacleAvoidance(){
    pinMode(_trigPin, OUTPUT);
    for(int i = 0; i < _numEchoPins; i++){
        pinMode(*(_echoPins+i), INPUT);
    }
}


void CB_ObstacleAvoidance::printAllEchoPins(){
    Serial.print("Echo pins: ");
    Serial.print(getEchoPin(0));

    for(int i = 1; i < _numEchoPins; i++){
        Serial.print(", ");
        Serial.print(getEchoPin(i));
    }

    Serial.print("\n");
}

// ------------------------- Utilities ------------------------- //

int CB_ObstacleAvoidance::findDistance(int index){
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_trigPin, LOW);
    float duration = pulseIn(getEchoPin(index), HIGH);
    return duration * 0.034/2;
}