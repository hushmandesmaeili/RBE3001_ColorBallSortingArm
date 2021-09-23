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

EE_Pos = zeros(1, 3);

try     
        q(1, :) = pp.ik3001(p(:, 1));
        q(2, :) = pp.ik3001(p(:, 2));
        q(3, :) = pp.ik3001(p(:, 3));
        q(4, :) = pp.ik3001(p(:, 4));
        
        c = 1;
        pp.servo_jp(q(1, :));
        pause(2);
        tic
        
        while (c < 5)
            currentJointConfig = pp.measured_js(1, 0);
            if (abs(q(c, 1) - currentJointConfig(1, 1)) <= BOUND && abs(q(c, 2) - currentJointConfig(1, 2)) <= BOUND && abs(q(c, 3) - currentJointConfig(1, 3)) <= BOUND)
                if (c ~= 1)
                    currentJointConfig = pp.measured_js(1, 0);
                end
                c = c + 1;
            end
            
            if c == 2
                pp.servo_jp(q(2, :));
            elseif c == 3
                pp.servo_jp(q(3, :));
            elseif c == 4
                pp.servo_jp(q(4, :));
            end
           
            currentJointConfig = pp.measured_js(1, 0);
            currentPos = pp.position(currentJointConfig(1, :));
            EE_Pos = [EE_Pos; currentPos]; 
            
        end
        
        
        writematrix(EE_Pos, 'CubicMotion_3JointAngs_Trajectory.csv');
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()