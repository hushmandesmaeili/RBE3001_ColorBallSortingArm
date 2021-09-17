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

p1 = [0 0 0];
p2 = [0 0 0];
p3 = [0 0 0];
p = [p1; p2; p3; p1];
q = [0; 0; 0; 0];


try
        q(1) = ik3001(p(1));
        q(2) = ik3001(p(2));
        q(3) = ik3001(p(3));
        q(4) = ik3001(p(4));
        trajPlanner = Traj_Planner(q);
        
        c = 1;
        
        while (c < 5)
            currentPos = pp.measured_js()
            if (currentPos(1:3, 4) == 
                c = c + 1;
            end
        
            if c == 1
                pp.servo_jp(
            end
            
            if c == 2
                pp.servo_jp(q2);
            elseif c == 3
                pp.servo_jp(q3);
            elseif c == 4
                pp.servo_jp(q1);
            end
        end
        
        tic
        pp.servo_jp(q(1));
        toc
        pp.servo_jp(q(2));
        toc
        pp.servo_jp(q(3));
        toc
        pp.servo_jp(q(4));
        toc
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()