function output = sl_controller(u)
%SL_CONTROLLER Seven plant states, driver integral, speed, acceleration and regen flag.
persistent p
if isempty(p),p=project_parameters();end
[actuator,dz,auxiliary]=driver_controller(u(1:7),u(8),u(9:11),p);
output=[actuator;dz;auxiliary];
end
