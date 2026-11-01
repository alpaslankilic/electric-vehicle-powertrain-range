function plot_results(t,result,input,metrics,p,visible)
%PLOT_RESULTS Mission feasibility, battery accounting, consumption and motor limits.
colors=[.8 .32 .05;0 .36 .60];names={'Friction braking only','Regeneration + friction'};
[fig,layout]=new_figure('Forward mission tracking',2,visible);
ax=nexttile(layout);hold(ax,'on');plot(ax,t,3.6*input(:,1),':','Color',[.4 .4 .4], ...
    'LineWidth',1.3,'DisplayName','Speed request');
for mode=1:2,plot(ax,t,3.6*result{mode}.state(:,1),'Color',colors(mode,:),'LineWidth',1.6,'DisplayName',names{mode});end
events(ax,p);ylabel(ax,'Vehicle speed [km/h]');style(ax);
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
ax=nexttile(layout);stairs(ax,t,100*input(:,4),'Color',[.45 .2 .55],'LineWidth',1.6);
events(ax,p);ylabel(ax,'Road grade [%]');xlabel(ax,'Time [s]');style(ax);
save_figure(fig,'01_mission_and_grade',visible);
[fig,layout]=new_figure('Battery power and SOC',2,visible);
for panel=1:2
    ax=nexttile(layout);hold(ax,'on');
    for mode=1:2
        value=result{mode}.control(:,5)/1000;if panel==2,value=100*result{mode}.state(:,3);end
        plot(ax,t,value,'Color',colors(mode,:),'LineWidth',1.5,'DisplayName',names{mode});
    end
    events(ax,p);style(ax);
    if panel==1,ylabel(ax,'Battery DC power [kW]');legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
    else,ylabel(ax,'State of charge [%]');xlabel(ax,'Time [s]');end
end
save_figure(fig,'02_battery_power_and_soc',visible);
[fig,layout]=new_figure('Consumption and range',2,visible);
ax=nexttile(layout);bars=bar(ax,[metrics.consumption_Wh_km],'FaceColor','flat');bars.CData=colors;
set(ax,'XTickLabel',{'Friction only','With regeneration'});ylabel(ax,'Chemical energy use [Wh/km]');style(ax);
title(ax,'(a) Same illustrative forward-driven mission','FontWeight','normal');
speed=(30:5:130).';consumption=zeros(size(speed));
for k=1:numel(speed)
    v=speed(k)/3.6;force=road_load(v,0,p);torque=force*p.wheelRadius/(p.gearRatio*p.gearEfficiency);
    state=p.initialState;state(1:2)=[v;torque];[~,current]=power_flow(state,p);
    consumption(k)=(330+60*state(3))*current/speed(k);
end
range=p.usableEnergy_kWh*1000./consumption;
ax=nexttile(layout);plot(ax,speed,range,'Color',colors(2,:),'LineWidth',1.8);
xlabel(ax,'Constant vehicle speed [km/h]');ylabel(ax,'Estimated range [km]');style(ax);
title(ax,'(b) Flat-road estimate at the initial battery SOC','FontWeight','normal');
writetable(table(speed,consumption,range,'VariableNames',{'speed_km_h','consumption_Wh_km','range_km'}),'results/constant_speed.csv');
save_figure(fig,'03_consumption_and_range',visible);
[fig,layout]=new_figure('Motor torque-power envelope',1,visible);ax=nexttile(layout);hold(ax,'on');
omega=linspace(1,p.maximumMotorSpeed,500);limit=min(p.peakTorque,p.peakPower./omega);
plot(ax,omega,limit,'--','Color',[.4 .4 .4],'LineWidth',1.4,'DisplayName','Torque-power envelope');
plot(ax,omega,-limit,'--','Color',[.4 .4 .4],'LineWidth',1.4,'HandleVisibility','off');
for mode=1:2
    state=result{mode}.state;
    scatter(ax,p.gearRatio*state(1:10:end,1)/p.wheelRadius,state(1:10:end,2),8,colors(mode,:), ...
        'filled','DisplayName',names{mode});
end
xlabel(ax,'Motor speed [rad/s]');ylabel(ax,'Motor shaft torque [N m]');style(ax);
xlim(ax,[0,p.maximumMotorSpeed]);
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');save_figure(fig,'04_motor_operating_points',visible);
end

function events(ax,p)
xlim(ax,[0,p.stopTime]);xline(ax,p.gradeTime,'--','Grade applied','Color',[.45 .2 .55], ...
    'LabelOrientation','horizontal','FontSize',10,'HandleVisibility','off');
xline(ax,p.flatTime,':','Grade removed','Color',[.45 .2 .55], ...
    'LabelOrientation','horizontal','FontSize',10,'HandleVisibility','off');
limits=ylim(ax);ylim(ax,[limits(1),limits(2)+.18*diff(limits)]);
end

function [fig,layout]=new_figure(name,panels,visible)
state='off';if visible,state='on';end
fig=figure('Name',name,'Visible',state,'Color','w','Position',[100 60 980 440+160*(panels-1)]);
set(fig,'DefaultAxesColorOrder',[0 .36 .60;.8 .32 .05;.2 .55 .35;.5 .3 .65]);
layout=tiledlayout(fig,panels,1,'TileSpacing','compact','Padding','loose');
end

function style(ax)
set(ax,'FontName','Arial','FontSize',12,'Box','off','TickDir','out', ...
    'YGrid','on','GridAlpha',.12,'LineWidth',.8);
end

function save_figure(fig,name,visible)
exportgraphics(fig,fullfile('results',[name '.png']),'Resolution',200,'Padding',8,'Units','points');
exportgraphics(fig,fullfile('results',[name '.pdf']),'ContentType','vector','Padding',8,'Units','points');
if ~visible,close(fig);end
end
