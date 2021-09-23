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

pp.closeGripper();

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

BOUND = 8.5;
NUM_POINTS = 10;

try     
        q(1, :) = pp.ik3001(p(:, 1));
        q(2, :) = pp.ik3001(p(:, 2));
        q(3, :) = pp.ik3001(p(:, 3));
        q(4, :) = pp.ik3001(p(:, 4));
        
        %Point 1 --> Point 2 --> Point 3 --> Point 1
        
%         joints_setpoints = zeros(NUM_POINTS, 3);
%         T = zeros(1, NUM_POINTS);
        A_set = zeros(4,3,3);
        
        
        for x = 1:3
            
            if x ~= 3
                %a coefficients for joint 1, 2 and 3.
                A_J1 = trajPlan.cubic_traj(t0, tf, q(x, 1), q(x+1, 1), v0, vf);
                A_J2 = trajPlan.cubic_traj(t0, tf, q(x, 2), q(x+1, 2), v0, vf);
                A_J3 = trajPlan.cubic_traj(t0, tf, q(x, 3), q(x+1, 3), v0, vf);
                
            else
                %a coefficients for joint 1, 2 and 3.
                A_J1 = trajPlan.cubic_traj(t0, tf, q(x, 1), q(1, 1), v0, vf);
                A_J2 = trajPlan.cubic_traj(t0, tf, q(x, 2), q(1, 2), v0, vf);
                A_J3 = trajPlan.cubic_traj(t0, tf, q(x, 3), q(1, 3), v0, vf);
            end

            A_set(:,1:3, x) = [A_J1 A_J2 A_J3];

        end
        
        T = zeros(1,1);
        EE_Pos = zeros(1, 3);
        EE_Vel = zeros(1, 3);
        EE_Acc = zeros(1, 3);
        accumTime = 0;

        pp.servo_jp(q(1, :));
        pause(1);
        
        for i = 1:3
            disp('Doing');
            currentJointConfig = pp.measured_js(1, 0);
            currentPos = pp.position(currentJointConfig(1, :));
            tic
            while (norm(transpose(p(:, i+1)) - currentPos) > BOUND)
                setpoint = trajPlan.getSetpoint(toc, A_set(:, 1, i), A_set(:, 2, i), A_set(:, 3, i));
                pp.servo_jp(setpoint);
                currentJointConfig = pp.measured_js(1, 0);
                currentPos = pp.position(currentJointConfig(1, :));
                
                %Recording data
                EE_Pos = [EE_Pos; currentPos]; 
                T = [T; toc + accumTime];
            end
            accumTime = accumTime + toc;
        end
        
        velT = 0;
        accT = 0;
        
        %Deriving velocity for EE from EE position
        for i = 1:length(EE_Pos)
            
            if (i ~= 1)
                if (EE_Pos(i) ~= EE_Pos(i-1))
                    EE_Vel = [EE_Vel; (EE_Pos(i, 1:3) - EE_Pos(i-1, 1:3)) / (T(i) - T(i-1))];
                    velT = [velT; T(i)];
                end 
            else
                EE_Vel(i, 1:3) = 0;
            end  
        end  
        
        %Deriving acceleration for EE from EE velocity
        for i = 1:length(EE_Vel)
            
            if (i ~= 1)
                if (EE_Vel(i) ~= EE_Vel(i-1))
                    EE_Acc = [EE_Acc; (EE_Vel(i, 1:3) - EE_Vel(i-1, 1:3)) / (T(i) - T(i-1))];
                    accT = [accT; T(i)];
                end 
            else
                EE_Acc(i, 1:3) = 0;
            end  
        end  
        
        
        %Plot of Tip position vs Time
        subplot(3, 1, 1)
        hold on
        plot(T, EE_Pos(:, 1), 'r-') %End Effector X Position
        plot(T, EE_Pos(:, 2), 'g-') %End Effector Y Position
        plot(T, EE_Pos(:, 3), 'b-') %End Effector Z Position
        legend('End Effector X Position', 'End Effector Y Position', 'End Effector Z Position')
        title("Plot of End Effector Position vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Position (mm)')
        hold off
        
        %Plot of Tip Velocity vs Time
        subplot(3, 1, 2)
        hold on
        plot(velT, EE_Vel(:, 1), 'r-') %End Effector X Vel
        plot(velT, EE_Vel(:, 2), 'g-') %End Effector Y Vel
        plot(velT, EE_Vel(:, 3), 'b-') %End Effector Z Vel
        legend('End Effector X Velocity', 'End Effector Y Velocity', 'End Effector Z Velocity')
        title("Plot of End Effector Velocity vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Velocity (mm/s)')
        hold off
        
        %Plot of Tip Acceleration vs Time
        subplot(3, 1, 3)
        hold on
        plot(accT, EE_Acc(:, 1), 'r-') %End Effector X Acceleration
        plot(accT, EE_Acc(:, 2), 'g-') %End Effector Y Acceleration
        plot(accT, EE_Acc(:, 3), 'b-') %End Effector Z Acceleration
        legend('End Effector X Acceleration', 'End Effector Y Acceleration', 'End Effector Z Acceleration')
        title("Plot of End Effector Acceleration vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Acceleration (mm/s^2)')
        hold off      
        
        
        writematrix(EE_Pos, 'CubicMotion_JointSpaceAngs_Trajectory.csv');
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()