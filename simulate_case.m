function [t,y,control] = simulate_case(p,stopTime)
%SIMULATE_CASE Piecewise integration of the forward-facing vehicle model.
t=(0:p.sampleTime:stopTime).';y=zeros(numel(t),8);state=[p.initialState;p.initialControlState];
knots=unique([0,p.rampStart,p.rampStart+p.accelerationDuration,p.gradeTime,p.fastTime, ...
    p.fastTime+p.accelerationDuration,p.flatTime,p.brakeTime,p.brakeTime+p.brakeDuration,stopTime]);
knots=knots(isfinite(knots)&knots<=stopTime);options=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',p.sampleTime);
for k=1:numel(knots)-1
    left=knots(k);right=knots(k+1);
    solution=ode45(@(time,x) closed_loop(min(right-1e-10,max(left,time)),x,p),[left,right],state,options);
    selected=t>=left & t<=right;y(selected,:)=deval(solution,t(selected)).';state=solution.y(:,end);
end
control=zeros(numel(t),6);
for k=1:numel(t)
    [actuator,dz,auxiliary]=driver_controller(y(k,1:7).',y(k,8),experiment_input(t(k),p),p);
    control(k,:)=[actuator;dz;auxiliary].';
end
end

function dy=closed_loop(t,y,p)
input=experiment_input(t,p);[actuator,dz]=driver_controller(y(1:7),y(8),input(1:3),p);
dy=[vehicle_rhs(y(1:7),actuator,input(4),p);dz];
end
