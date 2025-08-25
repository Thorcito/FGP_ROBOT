function PlotCtrlData( name, level, fig)
%PLOTCTRLDATA Summary of this function goes here
%   Detailed explanation goes here

y=load(name);

x1=y(:,1:4);
x2=rad2deg(y(:,5:end));

x2ms=[x1 x2];

x=x2ms(1:4:end,:);



% close all

joint_idx=5;
jointd_idx=23;

jointdp_idx=29;
jointdpp_idx=35;


djoint_idx=41;
dpjoint_idx=47;

torque_idx=65;

t=x(:,4)-x(1,4);

% pos=[-1854  70  1800  1001];
pos=[0  70  1800  1001];

j=fig;

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        %         plot(t,x(:,joint_idx+(i)),'b -',t,x(:,jointd_idx+i),'r -',t,x(:,torque_idx+i), 'm -')
        plot(t,x(:,joint_idx+(i)),'b -',t,x(:,jointd_idx+i),'r -')
        ns=sprintf('joint P %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('deg')
        grid on
    end
    j=j+1;
end

cut=1000;

cut=size(t)(1)-1

if(level>1)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    for i=0:5
%        subplot(6,1,i+1);
        subplot(3,2,i+1);
        plot(t(end-cut:end),x(end-cut:end,djoint_idx+(i)),'b -')
        ns=sprintf('joint Error Position%d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('deg')
        grid on
    end
    j=j+1;
    
end

if(level>2)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    for i=0:5
%        subplot(6,1,i+1);
        subplot(3,2,i+1);
        plot(t,x(:,dpjoint_idx+(i)),'b -')
        ns=sprintf('joint Error Velocity %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('deg/s')
        grid on
    end
    j=j+1;
end




end

