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
    
  pp.interpolate_jp([45 0 0], 2000);
  
  pos_array = zeros(1, 3);
  time_array = zeros(1, 1);
  timestep_array = zeros(1, 1);
  constrained_timestep_array = zeros(1, 1);
  
  tic
  measuredArray = pp.measured_js(1, 0);
  while toc < 2 %make stop when arm has reached target
      measuredArray = pp.measured_js(1, 0);
      pos_array(end+1, :) = measuredArray(1, :);
      time_array(end+1, 1) = 1000*toc;
      timestep_array(end+1, 1) = time_array(end, 1) - time_array(end-1, 1);
      if timestep_array(end, 1) < 1
          constrained_timestep_array(end+1, 1) = timestep_array(end, 1);
      end
  end
  
  
  outputMatrix = [time_array pos_array]; %combines matrices into one, with time being in the first column
  writematrix(outputMatrix, 'Time_Position.csv'); %outputs to .csv file
  
  subplot(5, 1, 1) %subplot 1 (top of column)
  plot(time_array(:, 1), pos_array(:, 1)) %plot time vs first column of position
  xlabel('Time [ms]') %x axis label
  ylabel('Position [degrees]') %y axis label
  title('Joint 1 Motion') %title of subplot
  
  subplot(5, 1, 2) %subplot 2 (second row in column), all other lines work same way as the previous subplot
  plot(time_array(:, 1), pos_array(:, 2))
  ylim([-25 25]);
  xlabel('Time [ms]')
  ylabel('Position [degrees]')
  title('Joint 2 Motion')
  
  subplot(5, 1, 3)
  plot(time_array(:, 1), pos_array(:, 3))
  ylim([-25 25]);
  xlabel('Time [ms]')
  ylabel('Position [degrees]')
  title('Joint 3 Motion')
  
  subplot(5, 1, 4)
  histogram(timestep_array);
  title('Histogram all packets')
  
  subplot(5, 1, 5)
  histogram(constrained_timestep_array);
  title('Histogram 0-5ms')
  
  
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()


