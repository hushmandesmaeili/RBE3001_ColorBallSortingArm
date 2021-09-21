classdef Traj_Planner
    %Traj_Planner Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        viaPoints;
    end
    
    methods
        
        function self = Traj_Planner()
            %Traj_Planner Construct an instance of this class
            %   Detailed explanation goes here
            
%             self = viaPoints;
        end
        
        %Solves for a cubic polynomial trajectory between two via-points
        %t0 parameter is initial time, tf is final time
        %p0 parameter is initial position, pf is final position
        %p0_dot is initial velocity, pf_dot is final velocity
        function A = cubic_traj(self, t0, tf, p0, pf, p0_dot, pf_dot)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            M = [1, t0, t0^2,   t0^3;
                 0, 1,  2*t0,   3*t0^2;
                 1, tf, tf^2,   tf^3;
                 0, 1,  2*tf,   3*tf^2];
             
            constraints = [p0; p0_dot; pf; pf_dot];
            
            A = M\constraints;
        end
        
        function jointsSetpoints = genJointSetpoints(self, t, A_J1, A_J2, A_J3)
            
            jointsSetpoints = zeros(1, 3);
            
            for i = 1:length(t)
                jointsSetpoints(i, 1:3) = [1 t(i) t(i)^2 t(i)^3]*[A_J1 A_J2 A_J3];
            end
        end
        
        function setpoint = getSetpoint(self, t, A_J1, A_J2, A_J3)
            
            setpoint = zeros(1, 3);
            
            setpoint(1, 1:3) = [1 t t^2 t^3]*[A_J1 A_J2 A_J3];
        end
    end
end

