clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 

% % Creates Frame3D object
% frame = Frame3D();

% mod = Model(pp, frame);

p1 = [100; 0; 195];
p2 = [50; 0; 150];
p3 = [60; 60; 60];
p = [p1 p2 p3 p1];
q = zeros(4,3);

t0 = 0;
tf = 2.4;
v0 = 0;
vf = 0;

try
        q(1, :) = pp.ik3001(p(:, 1));
        q(2, :) = pp.ik3001(p(:, 2));
        q(3, :) = pp.ik3001(p(:, 3));
        q(4, :) = pp.ik3001(p(:, 4));
%         trajPlan = Traj_Planner(q);
        
        %Point 1 --> Point 2
        
        %a coefficients for joint 1, 2 and 3.
        A_J1 = trajPlan.cubic_traj(t0, tf, q(1, 1), q(2, 1), v0, vf);
        A_J2 = trajPlan.cubic_traj(t0, tf, q(1, 2), q(2, 2), v0, vf);
        A_J3 = trajPlan.cubic_traj(t0, tf, q(1, 3), q(2, 3), v0, vf);
        
        t = linspace(t0, tf, 10);
        
        joints_setpoints = zeros(10, 3);
        joints_velpoints = zeros(10, 3);
        joints_accpoints = zeros(10, 3);
        
        EE_Pos = zeros(1,3);
        EE_Vel = zeros(1,3);
        EE_Vel = zeros(1,3);
        
        %Generate intermediate joint space configurations using  
        %cubic interpolation calculation at than 10 time steps for one edge
        for i = 1:10
            joints_setpoints(i, 1:3) = [1 t(i) t(i)^2 t(i)^3]*[A_J1 A_J2 A_J3];
            joints_velpoints(i, 1:3) = [1 t(i) t(i)^2]*[A_J1(2:end) A_J2(2:end) A_J3(2:end)];
            joints_accpoints(i, 1:3) = [1 t(i)]*[A_J1(3:end) A_J2(3:end) A_J3(4:end)];
            
            EE_Pos(1,:) = pp.position(joints_setpoints(i, 1:3));
            EE_Vel(1,:) = pp.position(joints_velpoints(i, 1:3));
            EE_Acc(1,:) = pp.position(joints_accpoints(i, 1:3));
        end
        
        %Send to the robot joint set points 
        for i = 1:10
            servo_jp(joints_setpoints(i, :));
        end
        
        
        %Plot of Tip position vs Time
        subplot(3, 1, 1)
        hold on
        plot(t, EE_Pos(:, 1), 'r-') %End Effector X Position
        plot(t, EE_Pos(:, 2), 'g-') %End Effector Y Position
        plot(t, EE_Pos(:, 3), 'b-') %End Effector Z Position
        legend('End Effector X Position', 'End Effector Y Position', 'End Effector Z Position')
        title("Plot of End Effector Position vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Position (mm)')
        hold off
        
        %Plot of Tip Velocity vs Time
        subplot(3, 1, 1)
        hold on
        plot(t, EE_Vel(:, 1), 'r-') %End Effector X Vel
        plot(t, EE_Vel(:, 2), 'g-') %End Effector Y Vel
        plot(t, EE_Vel(:, 3), 'b-') %End Effector Z Vel
        legend('End Effector X Velocity', 'End Effector Y Velocity', 'End Effector Z Velocity')
        title("Plot of End Effector Velocity vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Velocity (mm/s)')
        hold off
        
        %Plot of Tip Acceleration vs Time
        subplot(3, 1, 1)
        hold on
        plot(t, EE_Acc(:, 1), 'r-') %End Effector X Acceleration
        plot(t, EE_Acc(:, 2), 'g-') %End Effector Y Acceleration
        plot(t, EE_Acc(:, 3), 'b-') %End Effector Z Acceleration
        legend('End Effector X Acceleration', 'End Effector Y Acceleration', 'End Effector Z Acceleration')
        title("Plot of End Effector Acceleration vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Acceleration (mm/s^2)')
        hold off
        
        
%         %Point 1 --> Point 2 --> Point 3 --> Point 1
%         
%         for x = 1:3
%             
%             t = linspace(t0, tf, 10);
%         
%             joints_setpoints = zeros(10, 3);
%             
%             if x == 3
%                 %a coefficients for joint 1, 2 and 3.
%                 A_J1 = trajPlan.cubic_traj(t0, tf, q(i, 1), q(i+1, 1), v0, vf);
%                 A_J2 = trajPlan.cubic_traj(t0, tf, q(i, 2), q(i+1, 2), v0, vf);
%                 A_J3 = trajPlan.cubic_traj(t0, tf, q(i, 3), q(i+1, 3), v0, vf);
%                 
%             else
%                 %a coefficients for joint 1, 2 and 3.
%                 A_J1 = trajPlan.cubic_traj(t0, tf, q(i, 1), q(1, 1), v0, vf);
%                 A_J2 = trajPlan.cubic_traj(t0, tf, q(i, 2), q(1, 2), v0, vf);
%                 A_J3 = trajPlan.cubic_traj(t0, tf, q(i, 3), q(1, 3), v0, vf);
%             end
% 
%             %Generate intermediate joint space configurations using  
%             %cubic interpolation calculation at 10 time steps for one edge
%             for i = 1:10
%                 joints_setpoints(i, 1:3) = [1 t(i) t(i)^2 t(i)^3]*[A_J1 A_J2 A_J3];
%             end
% 
%             %Send to the robot joint set points 
%             for i = 1:10
%                 servo_jp(joints_setpoints(i, :));
%             end
% 
%         end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()