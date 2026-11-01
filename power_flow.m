function [batteryPower,current,wheelForce,motorPower] = power_flow(x,p)
%POWER_FLOW Direction-aware gearbox and drive losses plus a resistive battery.
speed=max(0,x(1));torque=x(2);motorSpeed=p.gearRatio*speed/p.wheelRadius;
efficiency=max(.75,min(.95,.93-.05*(motorSpeed/p.maximumMotorSpeed-.6)^2 ...
    -.07*(abs(torque)/p.peakTorque-.5)^2));
motorPower=torque*motorSpeed;
wheelForce=torque*p.gearRatio/p.wheelRadius;
if torque>=0,wheelForce=wheelForce*p.gearEfficiency;else,wheelForce=wheelForce/p.gearEfficiency;end
electricalPower=motorPower*efficiency;
if motorPower>=0,electricalPower=motorPower/efficiency;end
batteryPower=electricalPower+p.auxiliaryPower;voltage=330+60*x(3);
discriminant=voltage^2-4*p.batteryResistance*batteryPower;
assert(discriminant>0,'Battery power request is infeasible.');
current=2*batteryPower/(voltage+sqrt(discriminant));
end
