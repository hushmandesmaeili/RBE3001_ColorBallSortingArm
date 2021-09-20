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

pp.closeGripper();

BOUND = 1.7;

p1 = [100; 0; 195];
p2 = [50; 0; 150];
p3 = [60; 60; 60];
p = [p1 p2 p3 p1];
q = zeros(4,3);


try
        
        q(1, :) = pp.ik3001(p(:, 1));
        q(2, :) = pp.ik3001(p(:, 2));
        q(3, :) = pp.ik3001(p(:, 3));
        q(4, :) = pp.ik3001(p(:, 4));
%         trajPlanner = Traj_Planner(q);
        
        c = 1;
        pp.servo_jp(q(1, :));
        pause(2);
        tic
        
        while (c < 5)
            currentJointConfig = pp.measured_js(1, 0);
            if (abs(q(c, 1) - currentJointConfig(1, 1)) <= BOUND && abs(q(c, 2) - currentJointConfig(1, 2)) <= BOUND && abs(q(c, 3) - currentJointConfig(1, 3)) <= BOUND)
                if (c ~= 1)
%                     toc
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
            
        end
        
%         tic
%         pp.servo_jp(q(1));
%         toc
%         pp.servo_jp(q(2));
%         toc
%         pp.servo_jp(q(3));
%         toc
%         pp.servo_jp(q(4));
%         toc
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()