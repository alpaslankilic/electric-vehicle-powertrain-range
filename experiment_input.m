function input = experiment_input(t,p)
%EXPERIMENT_INPUT Separate speed changes, a road-grade disturbance, then braking.
[fraction,slope]=ramp(t-p.rampStart,p.accelerationDuration);
speed=p.citySpeed*fraction;acceleration=p.citySpeed*slope;
if t>=p.fastTime
    [fraction,slope]=ramp(t-p.fastTime,p.accelerationDuration);
    speed=p.citySpeed+(p.fastSpeed-p.citySpeed)*fraction;
    acceleration=(p.fastSpeed-p.citySpeed)*slope;
end
if t>=p.brakeTime
    [fraction,slope]=ramp(t-p.brakeTime,p.brakeDuration);
    speed=p.fastSpeed*(1-fraction);acceleration=-p.fastSpeed*slope;
end
grade=p.grade*(t>=p.gradeTime && t<p.flatTime);
input=[speed;acceleration;p.regenMode;grade];
end

function [fraction,slope]=ramp(t,duration)
fraction=min(1,max(0,t/duration));slope=(t>=0 && t<duration)/duration;
end
