function p = project_parameters()
%PROJECT_PARAMETERS Illustrative midsize EV; no measured vehicle calibration is assumed.
p.mass=1600;p.gravity=9.81;p.rollingCoefficient=.01;p.dragArea=.60;p.airDensity=1.2;
p.wheelRadius=.31;p.gearRatio=9;p.gearEfficiency=.97;
p.peakTorque=310;p.peakPower=150e3;p.maximumMotorSpeed=1500;p.motorTimeConstant=.07;
p.capacityAh=60e3/360;p.batteryResistance=.08;p.auxiliaryPower=400;p.usableEnergy_kWh=54;
p.driverKp=900;p.driverKi=250;p.driverTracking=2;
p.citySpeed=15;p.fastSpeed=25;p.rampStart=.1;p.accelerationDuration=4;p.brakeDuration=8;
p.grade=.03;p.gradeTime=Inf;p.fastTime=Inf;p.flatTime=Inf;p.brakeTime=Inf;
p.sampleTime=.02;p.sizingDuration=20;p.stopTime=20;p.regenMode=1;
p.initialState=[0;0;.8;0;0;0;0];p.initialControlState=0;
p.modelName='ev_powertrain';
end
