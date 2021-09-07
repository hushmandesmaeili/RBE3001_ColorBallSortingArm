classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962;
        endMotionSetPos;
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(packet, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(packet, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(packet.GRIPPER_ID);
                packet.myHIDSimplePacketComs.writeBytes(intid, ds, packet.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(packet)
            packet.writeGripper(180);
        end  
        
        % Closes the gripper
        function closeGripper(packet)
            packet.writeGripper(0);
        end
        
        % Takes a 1x3 array of joint values in degrees to be sent directly
        % to the actuators and bypasses interpolation
        function servo_jp(pp, q)
            SERV_ID = 1848;
            packet = zeros(15, 1, 'single');
            packet(1) = 0;    %one second time
            packet(2) = 0;       %linear interpolation
            packet(3) = q(1);
            packet(4) = q(2);       % Second link to 0
            packet(5) = q(3);       % Third link to 0
            
            pp.endMotionSetPos = q;

            % Send packet to the server and get the response      
            %pp.write sends a 15 float packet to the micro controller
            pp.write(SERV_ID, packet)
        end

        % Takes a 1x3 array joint values and interpolation time in ms
        function interpolate_jp(pp, q, t)
            tic
            SERV_ID = 1848;
            packet = zeros(15, 1, 'single');
            packet(1) = t;
            packet(2) = 0;
            packet(3) = q(1);
            packet(4) = q(2);
            packet(5) = q(3);
            
            pp.endMotionSetPos = q;

            pp.write(SERV_ID, packet);
            toc

        end
        
        % Takes two bool values, GETPOS and GETVEL. Returns only requested
        % date, and set rest to zero. 
        % Returns a 1x3 array that contains current joint positions in
        % degrees (1st row) and/or current velocities (2nd row)
        function current = measured_js(pp, GETPOS, GETVEL)
            current = zeros(2, 3);
            
            if GETPOS
               SERVER_ID_READ = 1910;
               returnPacket = pp.read(SERVER_ID_READ);
               current(1, :) = [returnPacket(3, 1) returnPacket(5, 1) returnPacket(7, 1)]; 
            end
            
            if GETVEL
               SERVER_ID_READ = 1822;
               returnPacket = pp.read(SERVER_ID_READ);
               current(2, :) = [returnPacket(3, 1) returnPacket(6, 1) returnPacket(9, 1)];
            end
        end
        
        %Returns 1x3 array that contains current joint set point positions
        %in degrees
        function currentSetPos = setpoint_js(pp)
            currentSetPos = zeros(1,3,'single'); %set output to zero 1x3 array
            packet = zeros(15, 1, 'single'); %set packet to zero 1x15 array
            SERV_ID_READ = 1910; %set id to 1910 -> the one for getting positions and setpoints    
            packet = pp.read(SERV_ID_READ); %set packet to read current robot positions and setpoints
            currentSetPos = [packet(2, 1) packet(4, 1) packet(6,1)]; %set output array to setpoints for each motor (3 total)
        end
        
        % Returns a 1x3 array that contains the end-of-motion joint setpoint 
        % positions in degrees
        function goal = goal_js(pp)
            goal = pp.endMotionSetPos;
        end
        
    end
    
end
