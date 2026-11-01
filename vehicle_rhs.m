function dx = vehicle_rhs(x,actuator,grade,p)
%VEHICLE_RHS Vehicle, motor lag, battery charge, distance, and energy counters.
[power,current,force]=power_flow(x,p);voltage=330+60*x(3);
acceleration=(force-actuator(2)-road_load(x(1),grade,p))/p.mass;
if x(1)<=0 && acceleration<0,acceleration=0;end
dx=[acceleration;(actuator(1)-x(2))/p.motorTimeConstant; ...
    -current/(3600*p.capacityAh);max(0,x(1));voltage*current; ...
    max(0,-power);actuator(2)*max(0,x(1))];
end
