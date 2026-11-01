function [p,timing] = plan_experiment(p)
%PLAN_EXPERIMENT Finish each speed/grade transition before issuing the next one.
first=recovery(p,p.rampStart+p.accelerationDuration,p.citySpeed,0);
p.gradeTime=first+p.sampleTime;second=recovery(p,p.gradeTime,p.citySpeed,p.grade);
p.fastTime=second+p.sampleTime;third=recovery(p,p.fastTime+p.accelerationDuration,p.fastSpeed,p.grade);
p.flatTime=third+p.sampleTime;fourth=recovery(p,p.flatTime,p.fastSpeed,0);
p.brakeTime=fourth+p.sampleTime;last=recovery(p,p.brakeTime+p.brakeDuration,0,0);
p.stopTime=ceil((last+.1*(last-p.brakeTime-p.brakeDuration))/p.sampleTime)*p.sampleTime;
timing=struct('city_settled_s',first,'grade_start_s',p.gradeTime,'grade_settled_s',second, ...
    'fast_start_s',p.fastTime,'fast_settled_s',third,'grade_end_s',p.flatTime, ...
    'flat_settled_s',fourth,'braking_start_s',p.brakeTime,'final_settled_s',last, ...
    'stop_time_s',p.stopTime,'tail_fraction',.1);
timing.criteria='Speed, motor torque and driver force: 2% phase excursion; standstill floors 0.02 m/s, 0.05 Nm, 1 N';
end

function time=recovery(p,start,speed,grade)
force=road_load(speed,grade,p);torque=force*p.wheelRadius/(p.gearRatio*p.gearEfficiency);
latest=zeros(1,2);
for mode=0:1
    p.regenMode=mode;[t,y,u]=simulate_case(p,start+p.sizingDuration);
    selected=find(t>=start);deviation=[y(selected,1),y(selected,2),u(selected,4)]-[speed,torque,force];
    tolerance=max(.02*max(abs(deviation),[],1),[.02,.05,1]);
    last=find(any(abs(deviation)>tolerance,2),1,'last');
    assert(~isempty(last)&&last<numel(selected)-10,'Vehicle sizing interval did not capture settling.');
    latest(mode+1)=t(selected(last+1));
end
time=max(latest);
end
