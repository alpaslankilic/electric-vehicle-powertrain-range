function inputSignal = input_signal(p)
%INPUT_SIGNAL Sample smooth trajectories and preserve jumps with repeated times.
changes=[p.rampStart,p.rampStart+p.accelerationDuration,p.gradeTime,p.fastTime,p.fastTime+p.accelerationDuration,p.flatTime,p.brakeTime,p.brakeTime+p.brakeDuration];
changes=changes(isfinite(changes)&changes>0&changes<p.stopTime);
times=unique(round([(0:p.sampleTime:p.stopTime),changes,p.stopTime],12));
width=numel(experiment_input(0,p));inputSignal=zeros(2*numel(times),width+1);row=0;
for k=1:numel(times)
    time=times(k);
    if any(abs(changes-time)<1e-10)
        row=row+1;inputSignal(row,:)=[time,experiment_input(max(0,time-1e-9),p).'];
    end
    sample=time;if any(abs(changes-time)<1e-10),sample=time+1e-9;end
    row=row+1;inputSignal(row,:)=[time,experiment_input(sample,p).'];
end
inputSignal=inputSignal(1:row,:);
end
