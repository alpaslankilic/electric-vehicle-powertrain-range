function dx = sl_plant(u)
%SL_PLANT Seven states, motor/brake requests, then the independent road grade.
persistent p
if isempty(p),p=project_parameters();end
dx=vehicle_rhs(u(1:7),u(8:9),u(10),p);
end
