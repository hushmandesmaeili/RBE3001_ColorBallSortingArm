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

% Creates Frame3D object
frame = Frame3D();
mod = Model(pp, frame);

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
        
        %Point 1 --> Point 2 --> Point 3 --> Point 1
        
%         joints_setpoints = zeros(NUM_POINTS, 3);
%         T = zeros(1, NUM_POINTS);
        A_set = zeros(4,3,3);
        
        
        for x = 1:3
            
            if x ~= 3
                %a coefficients for joint 1, 2 and 3.
                A_J1 = trajPlan.cubic_traj(t0, tf, p(1, x), p(1, x+1), v0, vf);
                A_J2 = trajPlan.cubic_traj(t0, tf, p(2, x), p(2, x+1), v0, vf);
                A_J3 = trajPlan.cubic_traj(t0, tf, p(3, x), p(3, x+1), v0, vf);
                
            else
                %a coefficients for joint 1, 2 and 3.
                A_J1 = trajPlan.cubic_traj(t0, tf, p(1, x), p(1, 1), v0, vf);
                A_J2 = trajPlan.cubic_traj(t0, tf, p(2, x), p(2, 1), v0, vf);
                A_J3 = trajPlan.cubic_traj(t0, tf, p(3, x), p(3, 1), v0, vf);
            end

            A_set(:,1:3, x) = [A_J1 A_J2 A_J3];

        end
        
        T = zeros(50,1);
        EE_Pos = zeros(1, 3);
        EE_Vel = zeros(1, 3);
        EE_Acc = zeros(1, 3);
        EE_LinVel = zeros(50,3);
        EE_AngVel = zeros(50,3);
        EE_LinVel_Mag = zeros(50,3);
        accumTime = 0;
        index = 1;

        pp.servo_jp(pp.ik3001(p1));
        pause(1);
        
        for i = 1:3
            disp('Doing');
            currentJointConfig = pp.measured_js(1, 0);
            currentPos = pp.position(currentJointConfig(1, :));
            tic
            while (norm(transpose(p(:, i+1)) - currentPos) > BOUND)
                setpoint = trajPlan.getSetpoint(toc, A_set(:, 1, i), A_set(:, 2, i), A_set(:, 3, i));
                pp.servo_jp(pp.ik3001(setpoint));
                currentJointConfig = pp.measured_js(1, 1);
                currentJointVel = currentJointConfig(2, :);
                currentPos = pp.position(currentJointConfig(1, :));
                
                %Live Plot
                P = pp.fdk3001(currentJointConfig(1, :), currentJointVel.');
                mod.plot_arm_fvk(currentJointConfig(1, :), P(1:3));
                drawnow;
                
                
                %Recording data
                EE_LinVel(index, 1:3) = P(1:3).';
                EE_AngVel(index, 1:3) = P(4:6).';
                EE_LinVel_Mag(index) = norm(EE_LinVel(index, 1:3));
                index = index + 1;
                T(index) = toc + accumTime;
            end
            accumTime = accumTime + toc;
        end
               
        %Plot of Linear Velocities vs Time
        subplot(3, 1, 1)
        hold on
        plot(T(1:index), EE_LinVel(1:index, 1), 'r-') %End Effector X Position
        plot(T(1:index), EE_LinVel(1:index, 2), 'g-') %End Effector Y Position
        plot(T(1:index), EE_LinVel(1:index, 3), 'b-') %End Effector Z Position
        legend('End Effector X Linear Velocity', 'End Effector Y Linear Velocity', 'End Effector Z Linear Velocity')
        title("Plot of End Effector Linear Velocity vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Linear Velocity')
        hold off
        
        %Plot of Linear Velocities vs Time
        subplot(3, 1, 2)
        hold on
        plot(T(1:index), EE_AngVel(1:index, 1), 'r-') %End Effector X Position
        plot(T(1:index), EE_AngVel(1:index, 2), 'g-') %End Effector Y Position
        plot(T(1:index), EE_AngVel(1:index, 3), 'b-') %End Effector Z Position
        legend('End Effector X Angular Velocity', 'End Effector Y Angular Velocity', 'End Effector Z Angular Velocity')
        title("Plot of End Effector Angular Velocity vs Time")
        xlabel('Time (s)')
        ylabel('End Effector Angular Velocity')
        hold off
        
        %Plot of Linear Velocities vs Time
        subplot(3, 1, 3)
        hold on
        plot(T(1:index), EE_LinVel_Mag(1:index), 'k-') %End Effector X Position
        title("Magnitude of End Effector Linear Velocity vs Time")
        xlabel('Time (s)')
        ylabel('Magnitude of End Effector Linear Velocity')
        hold off
          
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()