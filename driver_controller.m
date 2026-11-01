function [actuator,dz,auxiliary] = driver_controller(x,z,input,p)
%DRIVER_CONTROLLER Flat-road feedforward, PI tracking, motor limits and brake blending.
error=input(1)-x(1);
request=p.mass*input(2)+road_load(input(1),0,p)+p.driverKp*error+z;
motorSpeed=p.gearRatio*max(0,x(1))/p.wheelRadius;
limit=min(p.peakTorque,p.peakPower/max(1,motorSpeed));
if motorSpeed>=p.maximumMotorSpeed,limit=0;end
desired=request*p.wheelRadius/(p.gearRatio*p.gearEfficiency);
if request<0
    desired=request*p.wheelRadius*p.gearEfficiency/p.gearRatio;
    enable=input(3)*min(1,max(0,x(1)/2))*(x(3)<.95);
    desired=max(-limit*enable,desired*enable);
end
torque=min(limit,max(-limit,desired));
[batteryPower,current,wheelForce]=power_flow(x,p);
brake=0;if request<0,brake=min(8000,max(0,-request+wheelForce));end
achievable=torque*p.gearRatio*p.gearEfficiency/p.wheelRadius;
dz=p.driverKi*error;
if request>0,dz=dz+p.driverTracking*(achievable-request);end
if input(1)==0 && x(1)<.02,dz=-z/.2;end
actuator=[torque;brake];auxiliary=[request;batteryPower;current];
end
