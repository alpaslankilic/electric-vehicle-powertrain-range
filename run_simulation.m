function [t,state,control,error] = run_simulation(p)
%RUN_SIMULATION Compare the editable model with a separate piecewise ODE solve.
[t,reference]=simulate_case(p,p.stopTime);inputSignal=input_signal(p);
in=Simulink.SimulationInput(p.modelName);
in=in.setVariable('p',p,'Workspace',p.modelName);
in=in.setVariable('inputSignal',inputSignal,'Workspace',p.modelName);
in=in.setModelParameter('StopTime',num2str(p.stopTime,12),'Solver','ode45', ...
    'MaxStep',num2str(p.sampleTime,12),'RelTol','1e-8','AbsTol','1e-10', ...
    'OutputOption','SpecifiedOutputTimes','OutputTimes',mat2str(t.'));
out=sim(in);state=interp1(out.states.Time,out.states.Data,t,'pchip');
control=interp1(out.control.Time,out.control.Data,t,'pchip');
reference=reference(:,1:numel(p.initialState));scale=max(1,max(abs(reference),[],1));
error=max(abs((state-reference)./scale),[],'all');
assert(error<5e-5,'Scaled Simulink/ODE state mismatch: %.3g',error);
end
