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
            pp.interpolate_jp([45 45 45], 1000);
        elseif c == 2
            pp.interpolate_jp([0 0 0], 1000);
        elseif c == 3
            pp.interpolate_jp([0 -10 0], 1000);
        elseif c == 4
            pp.interpolate_jp([-20 0 0], 1000);
        elseif c == 5
            pp.interpolate_jp([35 0 30], 1000);
        end
        q = pp.measured_js(1,0);
        q = q(1,:);
        mod.plot_arm(q);
        drawnow;
    end
%     mod.plot_arm([0 0 0]);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()