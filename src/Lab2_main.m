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

try 
    pp.closeGripper();
    close all;
    tic
    T = zeros(5, 4, 4);
    c = 1;
    %While loop runs for 60 seconds
    %Improvement note: Make non-blocking code, without while loop
    while (toc < 7)
        g = floor(toc)
        disp(c)
        if floor(toc) > c
            c = c + 1;
        end
        
        if c == 1
            pp.servo_jp([45 45 45]);
            T(c,:,:) = pp.fk3001([45 45 45]);
        elseif c == 2
            pp.servo_jp([0 0 0]);
            T(c,:,:) = pp.fk3001([0 0 0]);
        elseif c == 3
            pp.servo_jp([0 -10 0]);
            T(c,:,:) = pp.fk3001([0 -10 0]);
        elseif c == 4
            pp.servo_jp([-20 0 0]);
            T(c,:,:) = pp.fk3001([-20 0 0]);
        elseif c == 5
            pp.servo_jp([35 0 30]);
            T(c,:,:) = pp.fk3001([35 0 30]);
        end
        q = pp.measured_js(1,0);
        q = q(1,:);
        mod.plot_arm(q);
        drawnow;
        writematrix(T, "part7Matrices.csv");
    end
%     mod.plot_arm([0 0 0]);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()