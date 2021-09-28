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

% p1 = [100; 0; 195];
p1 = [41; -113; 111];
p2 = [0; 0; 250];
p = [p1 p2];
q = zeros(4,3);

t0 = 0;
tf = 2.5;
v0 = 0;
vf = 0; 

trajPlan = Traj_Planner();

BOUND = 1.5;

try     
%         q(1, :) = pp.ik3001(p1);
%         q(2, :) = pp.ik3001(p2);
        
        %a coefficients for joint 1, 2 and 3.
        A_J1 = trajPlan.cubic_traj(t0, tf, p(1, 1), p(1, 2), v0, vf);
        A_J2 = trajPlan.cubic_traj(t0, tf, p(2, 1), p(2, 2), v0, vf);
        A_J3 = trajPlan.cubic_traj(t0, tf, p(3, 1), p(3, 2), v0, vf);
        
        T = zeros(1,1);
        EE_Pos = zeros(1, 3);
        JDet = zeros(1,1);
        accumTime = 0;

        pp.servo_jp(pp.ik3001(p1));
        pause(1);
        
        disp('Doing');
        currentJointConfig = pp.measured_js(1, 0);
        currentPos = pp.position(currentJointConfig(1, :));
        singularity = false;
        tic
        while (((p2(1) - currentPos(1) > BOUND) || (p2(2) - currentPos(2) > BOUND) || (p2(3) - currentPos(3) > BOUND)) && ~singularity)
            setpoint = trajPlan.getSetpoint(toc, A_J1, A_J2, A_J3);
            pp.servo_jp(pp.ik3001(setpoint));
            currentJointConfig = pp.measured_js(1, 0);
            q = currentJointConfig(1, :);
            currentPos = pp.position(q);
            
            if (pp.isCloseToSingularity(q))
                pp.EStop();
                singularity = true;
            end

            %Recording data
            EE_Pos = [EE_Pos; currentPos]; 
            JDet = [JDet; pp.calcJacobianDet(q)]; 
            T = [T; toc + accumTime];
            
        end
        
        
        %Plot of Tip position vs Time
        subplot(2, 1, 1)
        plot3(EE_Pos(2:end, 1), EE_Pos(2:end, 2), EE_Pos(2:end, 3));
        hold on
        legend('End Effector Position')
        title("Plot of End Effector Motion")
        xlabel('X (mm)')
        ylabel('Y (mm)')
        zlabel('Z (mm)')
        hold off
        
        %Plot of Tip Velocity vs Time
        subplot(2, 1, 2)
        hold on
        plot(T(2:end), JDet(2:end), 'b-') %Jacobian determinant vs time
        legend('Jacobian determinant')
        title("Plot of Jacobian Determinant vs Time")
        xlabel('Time (s)')
        ylabel('Jacobian Determinant')
        hold off  
        
%         writematrix(EE_Pos, 'CubicMotion_JointSpaceAngs_Trajectory.csv');
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()