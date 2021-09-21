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

graph = graphing("part3_ik_validation.csv", pp);



try
    %define these position vectors in mm
    p1 = [45 54 124];
    q1 = pp.ik3001(p1);
    p2 = [100 0 195];
    q2 = pp.ik3001(p2);
    p3 = [41 -113 111];
    q3 = pp.ik3001(p3);

    %Create empty matrix 10x4x4
    pp.closeGripper();
    pp.interpolate_jp([0 0 0], 1000);
    pause(2);
    
    close all;
    tic
    T = zeros(5, 4, 4);
    c = 1;
    %While loop runs for 60 seconds
    %Improvement note: Make non-blocking code, without while loop
    while (toc < 6)
%         g = floor(toc)
%         disp(c)
        if floor(toc) > c
            c = c + 1;
        end
        
        if c == 1
            pp.servo_jp(q1);
        elseif c == 2
            pp.servo_jp(q2);
        elseif c == 3
            pp.servo_jp(q3);
        elseif c == 4
            pp.servo_jp(q1);
        end
        
        q = pp.measured_js(1,0);
        q = q(1,:);
        mod.plot_arm(q);
        drawnow;
        graph.record();
    end
    
    graph.plotMotion(q1, q2, q3);
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()