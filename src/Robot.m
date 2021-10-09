classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        trajPlan;
        GRIPPER_ID = 1962;
        endMotionSetPos;
        endPoint;
        L0 = 55;    %Length of Link 0
        L1 = 40;    %Length of Link 1
        L2 = 100;   %Length of Link 2
        L3 = 100;   %Length of Link 3
        
        %1min, 1max; 2min 2max; 3min 3max;
        qlimdeg = [-90 90;
                -45 100;
                -90 63]; 
            
        qlim = deg2rad([-90 90;
                        -45 100;
                        -90 63]);
    end
    
    properties (Access = private)
       trajCoeffs_X;
       trajCoeffs_Y;
       trajCoeffs_Z;
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
            packet.trajPlan = Traj_Planner();
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
                    getReportheta1t(exception)
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
%             tic
            SERV_ID = 1848;
            packet = zeros(15, 1, 'single');
            packet(1) = t;
            packet(2) = 0;
            packet(3) = q(1);
            packet(4) = q(2);
            packet(5) = q(3);
            
            pp.endMotionSetPos = q;

            pp.write(SERV_ID, packet);
%             toc

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
        
        %lab 2 methods
        %forward kinematics
        %q = [theta d a alpha]
        function T = dh2mat(pp, q)
            T= [cosd(q(1)) -sind(q(1))*cosd(q(4)) sind(q(1))*sind(q(4)) q(3)*cosd(q(1));
                sind(q(1)) cosd(q(1))*cosd(q(4)) -cosd(q(1))*sind(q(4)) q(3)*sind(q(1));
                0 sind(q(4)) cosd(q(4)) q(2);
                0 0 0 1];
        end
        
        function T = dh2fk(pp, q)
           T = eye(4);
           for i = 1:size(q(:,1))
               T = T * pp.dh2mat(q(i, :));
           end
        end
        
        %Returns T04 HT matrix
        function T = fk3001(pp, jointConfig)
            
            T = [cos((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*cos((pi*(jointConfig(3) + 90))/180) - cos((pi*jointConfig(1))/180)*sin((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180), -sin((pi*jointConfig(1))/180), cos((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180) + cos((pi*jointConfig(1))/180)*cos((pi*(jointConfig(3) + 90))/180)*sin((pi*(jointConfig(2) - 90))/180), 100*cos((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180) + 100*cos((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*cos((pi*(jointConfig(3) + 90))/180) - 100*cos((pi*jointConfig(1))/180)*sin((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180);
                sin((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*cos((pi*(jointConfig(3) + 90))/180) - sin((pi*jointConfig(1))/180)*sin((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180),  cos((pi*jointConfig(1))/180), sin((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180) + sin((pi*jointConfig(1))/180)*cos((pi*(jointConfig(3) + 90))/180)*sin((pi*(jointConfig(2) - 90))/180), 100*sin((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180) + 100*sin((pi*jointConfig(1))/180)*cos((pi*(jointConfig(2) - 90))/180)*cos((pi*(jointConfig(3) + 90))/180) - 100*sin((pi*jointConfig(1))/180)*sin((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180);
                - cos((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180) - cos((pi*(jointConfig(3) + 90))/180)*sin((pi*(jointConfig(2) - 90))/180),                      0,                                             cos((pi*(jointConfig(2) - 90))/180)*cos((pi*(jointConfig(3) + 90))/180) - sin((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180),                                                              95 - 100*cos((pi*(jointConfig(2) - 90))/180)*sin((pi*(jointConfig(3) + 90))/180) - 100*cos((pi*(jointConfig(3) + 90))/180)*sin((pi*(jointConfig(2) - 90))/180) - 100*sin((pi*(jointConfig(2) - 90))/180);
                0, 0, 0, 1];          
            
        end
        
        %calculates task space based on joint angles
        %task space vector is a 3x1 vector [px; py; pz]
        function T = ik3001(self, ts)
            
            T = [NaN NaN NaN];

            %theta 1 is based on a triangle with known x and y
            theta1 = atan2(ts(2), ts(1));

            if ~(theta1 > self.qlim(1,1) && theta1 < self.qlim(1,2))
               error('Robot:bound', "theta 1 out of bounds");
            end
            
            d_1 = sqrt(ts(1)^2 + ts(2)^2 + (ts(3) - self.L1 - self.L0)^2);
            D_3 = (self.L3^2 + self.L2^2 - d_1^2)/(2*self.L2*self.L3);
            theta3_1 = atan2(D_3, sqrt(1-D_3^2));
            theta3_2 = atan2(D_3, -sqrt(1-D_3^2));
            
            D_2_1 = (self.L3*cos(theta3_1))/d_1;
            D_2_2 = (self.L3*cos(theta3_2))/d_1;
            a1_1 = atan2(D_2_1, sqrt(1-D_2_1^2));
            a1_2 = atan2(D_2_2, sqrt(1-D_2_2^2));
            %theoretical but unnecessary solutions
%             a1_3 = atan2(D_2_1, -sqrt(1-D_2_1^2));
%             a1_4 = atan2(D_2_2, -sqrt(1-D_2_2^2));
            
            a2 = atan2((ts(3) - self.L0 - self.L1), sqrt(ts(1)^2 + ts(2)^2));
            theta2_1 = pi/2 - a2 - a1_1;
            theta2_2 = pi/2 - a2 - a1_2;
%             theta2_3 = pi/2 - a2 - a1_3;
%             theta2_4 = pi/2 - a2 - a1_4;
            
%             theta = int16.empty(3,4);
%             theta(1,1) = theta1;
%             theta(2,1:4) = [theta2_1 theta2_2 theta2_3 theta2_4];
%             theta(3,1:2) = [
%             
%             theta = [theta1 NaN;
%                     theta2_1 theta2_2;
%                     theta3_1 theta3_2];

            theta = [theta1 NaN;
                    theta2_1 theta2_2;
                    theta3_1 theta3_2];

            %if values are out of bounds, remove them from array
            for i = 1:2
                if (theta(2, i) < self.qlim(2, 1)) || (theta(2, i) > self.qlim(2, 2))
                   theta(2, i) = NaN;
                end
            end
            
            for i = 1:2
                if (theta(3, i) < self.qlim(3, 1)) || (theta(3, i) > self.qlim(3, 2))
                   theta(3, i) = NaN;
                end
            end
            
            %if corresponding theta2 value is impossible, corresponding 3
            %values cannot be either
            if isnan(theta(2, 1)) 
               theta(3, 1) = NaN;
            end
            
            if isnan(theta(2, 2))
               theta(3, 2) = NaN;
            end
            
            %if corresponding theta 3 value is impossible, corresping
            %theta2 value cannot be either
            if isnan(theta(3, 1))
               theta(2, 1) = NaN;
%                theta(2, 3) = NaN;
            end
            
            if isnan(theta(3, 2))
               theta(2, 2) = NaN;
%                theta(2, 4) = NaN;
            end
            
            for i = 1:3
                for j = 1:2
                    if ~isnan(theta(i, j))
                       T(1, i) = theta(i, j);
                    end
                end
            end
            
            for i = 1:3
               if isnan(T(i))
                   error('Robot:bound', "joint values out of bounds");
               end
            end

            T = rad2deg(T);
        end
        
        %Returns T00 HT matrix given joint configuration
        function T = T00(pp, q)
            T = eye(4,4);
        end
        
        %Returns T01 HT matrix given joint configuration
        function T = T01(pp, q)
            T = [1, 0, 0, 0;
                 0, 1, 0, 0;
                 0, 0, 1, 55;
                 0, 0, 0, 1];

        end
        
        %Returns T02 HT matrix given joint configuration
        function T = T02(pp, q)
            T = [cos((pi*q(1, 1))/180),  0, -sin((pi*q(1, 1))/180),  0;
                   sin((pi*q(1, 1))/180),  0,  cos((pi*q(1, 1))/180),  0;
                   0, -1,                      0, 95;
                   0, 0, 0, 1];
        end
        
        %Returns T03 HT matrix given joint configuration
        function T = T03(pp, q)
            T = [cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -cos((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180), 100*cos((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                   sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180), -sin((pi*q(1, 1))/180)*sin((pi*(q(1, 2) - 90))/180),  cos((pi*q(1, 1))/180), 100*sin((pi*q(1, 1))/180)*cos((pi*(q(1, 2) - 90))/180);
                   -sin((pi*(q(1, 2) - 90))/180),                       -cos((pi*(q(1, 2) - 90))/180),                      0,                  95 - 100*sin((pi*(q(1, 2) - 90))/180);
                   0, 0, 0, 1];
        end
        
        %Takes data from measured_js() and returns a 4x4 homogeneous transformation
        %matrix based upon the current joint positions in degrees.
        function T = measured_cp(pp)
        
            q = pp.measured_js(1, 0);
            q = q(1, :);
            T = pp.fk3001(q);
        end
        
        %Returns HT matrix to get to current location of arm
        function T = setpoint_cp(pp)
            T = pp.fk3001(pp.setpoint_js());
        end
        
        % Takes data from goal_js() and returns a 4x4 homogeneous transformation
        % matrix based upon the commanded end of motion joint set point 
        % positions in degrees.pp.endMotionSetPos = endPoint
        function T = goal_cp(pp)
            T = pp.fk3001(pp.goal_js());
        end
        
        function T = position(pp, q)
            T = pp.fk3001(q);
            T = T(1:3, 4)';
        end
        
        %returns current xyz position of arm
        function p = currPosition(pp)
           q = pp.measured_js(1, 0);
           q = q(1, :);
           p = pp.position(q);
        end
        
        %Takes in a joint configuration (Theta1, theta2, theta3, returns Jacobian for that joint
        %configurationpp.endMotionSetPos = endPoint
        function J = jacob3001(pp, q)
            
           J = [ 
             (5*pi*sin((pi*q(1))/180)*sin((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*cos((pi*(q(3) + 90))/180))/9 - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(2) - 90))/180))/9,       - (5*pi*cos((pi*q(1))/180)*sin((pi*(q(2) - 90))/180))/9 - (5*pi*cos((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*q(1))/180)*cos((pi*(q(3) + 90))/180)*sin((pi*(q(2) - 90))/180))/9,     - (5*pi*cos((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*q(1))/180)*cos((pi*(q(3) + 90))/180)*sin((pi*(q(2) - 90))/180))/9
             (5*pi*cos((pi*q(1))/180)*cos((pi*(q(2) - 90))/180))/9 + (5*pi*cos((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*cos((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*q(1))/180)*sin((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9,       - (5*pi*sin((pi*q(1))/180)*sin((pi*(q(2) - 90))/180))/9 - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(3) + 90))/180)*sin((pi*(q(2) - 90))/180))/9,     - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*sin((pi*q(1))/180)*cos((pi*(q(3) + 90))/180)*sin((pi*(q(2) - 90))/180))/9
                                                                                                                                                                                                                                     0,                                                                  (5*pi*sin((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*(q(2) - 90))/180)*cos((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*(q(2) - 90))/180))/9,                                             (5*pi*sin((pi*(q(2) - 90))/180)*sin((pi*(q(3) + 90))/180))/9 - (5*pi*cos((pi*(q(2) - 90))/180)*cos((pi*(q(3) + 90))/180))/9
                                                                                                                                                                                                                                     0,                                                                                                                                                                                                               -sin((pi*q(1))/180),                                                                                                                                                     -sin((pi*q(1))/180)
                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                cos((pi*q(1))/180),                                                                                                                                                      cos((pi*q(1))/180)
                                                                                                                                                                                                                                     1,                                                                                                                                                                                                                                 0,                                                                                                                                                                       0];
        end
        
        %takes in current joint configuration, 
        %takes current joint velocities (3x1),
        %and returns current linear and angular velocities
        function P = fdk3001(pp, q, qdot)
            J = pp.jacob3001(q);
            P = J*qdot;
        end
        
        function d = calcJacobianDet(pp, q)
            J = pp.jacob3001(q);
            d = det(J(1:3, :));
        end
        
        %Method that determines whether robot is close to a singular
        %configuration. 
        function isClose = isCloseToSingularity(pp, q)
            d = pp.calcJacobianDet(q);
            
            if (d < 1.8)
                isClose = true;
            else
               isClose = false;
            end
        end
        
        
        function EStop(pp)
            currentJointConfig = pp.measured_js(1,0);
            pp.servo_jp(currentJointConfig(1, :));
            
%             disp('Error');
            
            error("Robot reached singular configuration. Motion has stopped.");
        end
        
        %takes in final position in task space, moves robot appropriately
        function trajMove(pp, t)
            setpoint = pp.trajPlan.getSetpoint(t, pp.trajCoeffs_X, pp.trajCoeffs_Y, pp.trajCoeffs_Z);
            if(setpoint(1, 1) > 160)
               error('Robot:xbound', "x too big"); 
            end
            q = pp.ik3001(setpoint);
            pp.servo_jp(q);
        end
        
        function setQuinticTraj(pp, endPoint, tf)
            tf = tf/1000;
%             pp.endPoint = pp.posAdjustOpenGripper(endPoint);
            pp.endPoint = endPoint;
            currentPos = pp.currPosition();
            
            pp.trajCoeffs_X = pp.trajPlan.quintic_traj(0, tf, currentPos(1), pp.endPoint(1), 0, 0, 0, 0);
            pp.trajCoeffs_Y = pp.trajPlan.quintic_traj(0, tf, currentPos(2), pp.endPoint(2), 0, 0, 0, 0);
            pp.trajCoeffs_Z = pp.trajPlan.quintic_traj(0, tf, currentPos(3), pp.endPoint(3), 0, 0, 0, 0);
            
        end
        
        function status = atEndPoint(pp)
            currentPos = pp.currPosition();
            BOUND = 8.5;
%             BOUND = [5 2 5];
            status = norm(pp.endPoint - currentPos) < BOUND;
%             status = (abs(pp.endPoint(1) - currentPos(1)) < BOUND(1)) && (abs(pp.endPoint(2) - currentPos(2)) < BOUND(2)) && (abs(pp.endPoint(3) - currentPos(3)) < BOUND(3));
        end
        
        %takes in xyz position of end of gripper, returns xyz position of
        %gripper open position
        function pOut = posAdjustOpenGripper(pp, pIn)
            pOut = zeros(1, 3);
            q = pp.measured_js(1,0);
            pOut(1) = pIn(1) - sind(-q(1))*15;
            pOut(2) = pIn(2) - cosd(-q(1))*15;
            pOut(3) = pIn(3);
        end
        
        % Returns interpolation time needed for given trajectory
        % Takes initial position, final position and calculates distance
        % between them. Also takes desired position. 
        % Speed in mm/s.
        function time = getInterpolationTime(pp, pf, speed)
            distance = norm(pf - pp.currPosition());
            time = distance / speed;
        end
        
    end
    
end
