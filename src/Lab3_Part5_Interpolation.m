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
close all;

% pp.closeGripper();

p1 = [45; 54; 24];
p2 = [100; 0; 195];
p3 = [41; -113; 111];
p = [p1 p2 p3 p1];
q = zeros(4,3);

t0 = 0;
tf = 2.5;
v0 = 0;
vf = 0; 

trajPlan = Traj_Planner();

BOUND = 2.0;
NUM_POINTS = 10;

try     
        q(1, :) = pp.ik3001(p(:, 1));
        q(2, :) = pp.ik3001(p(:, 2));
        q(3, :) = pp.ik3001(p(:, 3));
        q(4, :) = pp.ik3001(p(:, 4));
        
        %Point 1 --> Point 2
        
        %a coefficients for joint 1, 2 and 3.
        A_J1 = trajPlan.cubic_traj(t0, tf, q(1, 1), q(2, 1), v0, vf);
        A_J2 = trajPlan.cubic_traj(t0, tf, q(1, 2), q(2, 2), v0, vf);
        A_J3 = trajPlan.cubic_traj(t0, tf, q(1, 3), q(2, 3), v0, vf);
        
        t = linspace(t0, tf, NUM_POINTS);
        
        joints_setpoints = zeros(NUM_POINTS, 3);
        EE_Vel = zeros(NUM_POINTS, 3);
        EE_Acc = zeros(NUM_POINTS, 3);
        
        EE_Pos = zeros(1,3);
        EE_Vel = zeros(1,3);
        EE_Acc = zeros(1,3);
        
        %Generate intermediate joint space configurations using  
        %cubic interpolation calculation at NUM_POINTS time steps for one edge
        joints_setpoints = trajPlan.genJointSetpoints(t, A_J1, A_J2, A_J3);
        
        %Generate intermediate task space configurations and deriving  
        %velocity and acceleration for EE from EE position
        for i = 1:NUM_POINTS
%             joints_setpoints(i, 1:3) = [1 t(i) t(i)^2 t(i)^3]*[A_J1 A_J2 A_J3];
            EE_Pos(i, 1:3) = pp.position(joints_setpoints(i, 1:3));
            
            if (i ~= 1)
                EE_Vel(i, 1:3) =  (EE_Pos(i, 1:3) - EE_Pos(i-1, 1:3)) / (t(i) - t(i-1));
                
                EE_Acc(i, 1:3) =  (EE_Vel(i, 1:3) - EE_Vel(i-1, 1:3)) / (t(i) - t(i-1));
            else
                EE_Vel(i, 1:3) = 0;
                EE_Acc(i, 1:3) = 0;
            end  
        end
        
        
        
%         %Send to the robot joint set points
%         c = 0;
%         pp.servo_jp(joints_setpoints(1, 1:3));
%         c = c + 1;
%         while (c <= NUM_POINTS)
%             currentPos = pp.measured_js(1, 0);
%             if (abs(joints_setpoints(c, 1) - currentPos(1, 1)) <= BOUND && abs(joints_setpoints(c, 2) - currentPos(1, 2)) <= BOUND && abs(joints_setpoints(c, 3) - currentPos(1, 3)) <= BOUND)
%                 c = c + 1;
%                 if (c  <= NUM_POINTS)
%                     pp.servo_jp(joints_setpoints(c, 1:3));
%                 end
%             end
%         end
        
        disp("hi")
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
        subplot(3, 1, 2)
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
        subplot(3, 1, 3)
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
%             t = linspace(t0, tf, NUM_POINTS);
%         
%             joints_setpoints = zeros(NUM_POINTS, 3);
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