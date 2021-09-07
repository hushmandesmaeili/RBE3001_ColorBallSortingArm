%%
% RBE3001 - Laboratory 1 

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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
try
  
  pp.interpolate_jp([0 0 0], 1000);
  pause(2);
    
  pp.interpolate_jp([0 45 0], 2000);
  
  pos_array = zeros(1, 3);
  time_array = zeros(1, 1);
  
  tic
  measuredArray = pp.measured_js(1, 0);
  while toc < 2 %make stop when arm has reached target
      measuredArray = pp.measured_js(1, 0);
      pos_array(end+1, :) = measuredArray(1, :);
      time_array(end+1, 1) = 1000*toc;
  end
  
  disp("position array:");
  disp(pos_array);
  disp(time_array);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()


