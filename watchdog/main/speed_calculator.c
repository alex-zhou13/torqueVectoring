// Params
#define WHEEL_DIAM  0.5 // m
#define RATIO       3.6
#define AIR_DENSITY 1.23
#define DRAG_COEF   1
#define ROLL_COEF   0.02
#define FRONT_AREA  1.2
#define FRIC_COEF   0.8
#define WEIGHT      250
#define TIMESTEP    100 // ms
#define LENGTH      2 // m
#define WIDTH       1.5



void 



function zeroToSixty(step, weight, CD, area, secsHi, ratio, diameter, bias, fricCoef) {
  // INIT
  speed = 0; // meters per second
  time = 0; // Seconds
  acceleration = 0;
  force = 0; // Newtons
  high = 0; // 1 = high power, 0 = low power
  
  rpm = speed / (diameter*3.14159) * ratio * 60;
  staticFriction = weight*9.8*fricCoef*bias;
  
  while (speed <= 60*.44704) { // While less than 60 mph (but in m/s)
    if (time < secsHi){ // if within high power time window
      high = 1;
    } else {
      high = 0;
    }
    
    rpm = speed / (diameter*3.14159) * ratio * 60; // Calc RPM
    force = driveforce(ratio, diameter, rpm, high); // Find force
    if (force > staticFriction) { // Reduce to friction force (traction control lol)
      force = staticFriction;
    }
    
    acceleration = (force - 0.5*1.225*CD*area*speed^2 - weight * 9.8 * 0.014 / (diameter * .5))/weight; // Find acceleration from net force
    //                      aerodynamic drag            rolling resistance
    speed += acceleration*step; // Reimann sum that shit
    time += step; // step in time
  }
  return time;
}

// Appoximation of the torque curves provided by Emrax, returns force at WHEELS
function driveforce(ratio, diameter, rpm, high) { 
  motortorque = 0;
  // If not high power
  if (high == 0) {
    motortorque = 72 + 0.00635*rpm - 0.00000126*rpm*rpm;
  // If high power
  } else {
    if (rpm >= 0 && rpm < 1000) {
      motortorque = 140;
    } else if (rpm >= 1000 && rpm < 2000) {
      motortorque = 138; 
    } else if (rpm >= 2000 && rpm < 3000) {
      motortorque = 137; 
    } else if (rpm >= 3000 && rpm < 4000) {
      motortorque = 136; 
    } else if (rpm >= 4000 && rpm < 4600) {
      motortorque = 133; 
    } else if (rpm >= 4600 && rpm < 5000) {
      motortorque = 130; 
    } else if (rpm >= 5000 && rpm < 5200) {
      motortorque = 126; 
    } else if (rpm >= 5200 && rpm < 5400) {
      motortorque = 124; 
    } else if (rpm >= 5400 && rpm < 5600) {
      motortorque = 122; 
    } else if (rpm >= 5600 && rpm < 5800) {
      motortorque = 119; 
    } else if (rpm >= 5800 && rpm < 6000) {
      motortorque = 116; 
    } else {
      motortorque = 115; 
    }
  }
  // Torque multiplied by ratio, divided by radius
  return motortorque*ratio/(diameter/2);
}
  