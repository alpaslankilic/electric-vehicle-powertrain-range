function force = road_load(speed,grade,p)
%ROAD_LOAD Smooth low-speed rolling force, aerodynamic drag, and road grade.
force=p.mass*p.gravity*p.rollingCoefficient*tanh(speed/.3)+ ...
    .5*p.airDensity*p.dragArea*speed.*abs(speed)+p.mass*p.gravity*grade;
end
