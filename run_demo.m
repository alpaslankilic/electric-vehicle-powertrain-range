function report = run_demo(showFigures)
%RUN_DEMO Compare regenerative/friction braking on the same forward-driven mission.
if nargin==0,showFigures=true;end
root=fileparts(mfilename('fullpath'));previous=pwd;cleanup=onCleanup(@() cd(previous));cd(root);
clear sl_controller sl_plant
p=project_parameters();[p,timing]=plan_experiment(p);result=cell(1,2);metrics=cell(1,2);
for mode=0:1
    p.regenMode=mode;[t,state,control,error]=run_simulation(p);input=zeros(numel(t),4);
    for k=1:numel(t),input(k,:)=experiment_input(t(k),p).';end
    voltage=330+60*state(:,3);current=control(:,6);power=control(:,5);
    batteryResidual=max(abs(voltage.*current-p.batteryResistance*current.^2-power));
    assert(batteryResidual<1e-7,'Resistive battery power balance failed.');
    assert(all(state(:,3)>0 & state(:,3)<1),'SOC operating range failed.');
    assert(min(state(:,1))>-.001 && state(end,1)<.02,'Vehicle did not finish at standstill.');
    assert(max(abs(state(:,2)))<=p.peakTorque+1e-6,'Motor torque limit failed.');
    exactChemical=3600*p.capacityAh*(330*(p.initialState(3)-state(end,3))+ ...
        30*(p.initialState(3)^2-state(end,3)^2));
    assert(abs(state(end,5)-exactChemical)<.01,'Chemical-energy/SOC accounting failed.');
    distance=state(end,4)/1000;consumption=state(end,5)/3600/distance;
    metrics{mode+1}=struct('regeneration_enabled',logical(mode),'scaled_ode_max_error',error, ...
        'distance_km',distance,'chemical_energy_kWh',state(end,5)/3.6e6, ...
        'consumption_Wh_km',consumption,'estimated_range_km',p.usableEnergy_kWh*1000/consumption, ...
        'recovered_dc_energy_Wh',state(end,6)/3600,'friction_brake_energy_Wh',state(end,7)/3600, ...
        'battery_power_balance_error_W',batteryResidual);
    result{mode+1}=struct('state',state,'control',control);
    names={'time_s','speed_m_s','motor_torque_Nm','soc','distance_m','chemical_energy_J', ...
        'recovered_dc_energy_J','friction_brake_energy_J','torque_request_Nm','friction_request_N', ...
        'driver_integral_derivative','force_request_N','battery_power_W','battery_current_A'};
    writetable(array2table([t,state,control],'VariableNames',names),sprintf('results/regen_%d.csv',mode));
end
metrics=[metrics{:}];
assert(metrics(2).consumption_Wh_km<metrics(1).consumption_Wh_km,'Regeneration did not reduce energy consumption.');
report=struct('verification','PASS','cases',metrics,'stop_time_s',p.stopTime, ...
    'regeneration_saving_percent',100*(1-metrics(2).consumption_Wh_km/metrics(1).consumption_Wh_km));
write_json('results/parameters.json',p);write_json('results/timing.json',timing);write_json('results/metrics.json',report);
plot_results(t,result,input,metrics,p,showFigures);fprintf('EV verification PASS; saving %.2f percent.\n',report.regeneration_saving_percent);
end
