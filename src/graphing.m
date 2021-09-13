classdef graphing
    %collects data into a csv file
    %graphs various plots from a csv file
    
    
    properties
        csv;
        robot;
    end
    
    methods
        function self = graphing(file, bot)
            self.csv = file;
            self.robot = bot;
            writematrix(zeros(1, 7), self.csv);
        end
        
        %records current joint values and position to csv file
        function record(self)
            T = self.robot.measured_js(1, 0);
            T = T(1, :);
            q = self.robot.measured_cp();
            q = q(1:3, 4)';
            T = [toc T q];
            G = readmatrix(self.csv);
            writematrix([G;T], self.csv);
        end
        
        %plots all subplots
        function plotMotion(self, q1, q2, q3)
            mpMatrix = readmatrix(self.csv);
            mpMatrix = mpMatrix(2:end, :);
                        
            %Points of Triangle
            point1 = self.robot.position(q1);
            point2 = self.robot.position(q2);
            point3 = self.robot.position(q3);
            points = [point1; point2; point3];

            %Plot of Joint Angles vs Time
            subplot(4, 1, 1)
            hold on
            plot(mpMatrix(:, 1), mpMatrix(:, 2), 'r-') %Joint 1 Angle
            plot(mpMatrix(:, 1), mpMatrix(:, 3), 'g-') %Joint 2 Angle
            plot(mpMatrix(:, 1), mpMatrix(:, 4), 'b-') %Joint 3 Angle
            legend('Joint 1 Angle', 'Joint 2 Angle', 'Joint 3 Angle')
            title("Plot of Joint Angles vs Time")
            xlabel('Time (s)')
            ylabel('Joint Angle (deg)')
            hold off
            
            %Plot of Tip position vs Time
            subplot(4, 1, 2)
            hold on
            plot(mpMatrix(:, 1), mpMatrix(:, 5), 'r-') %End Effector X Position
            plot(mpMatrix(:, 1), mpMatrix(:, 7), 'b-') %End Effector Z Position
            legend('End Effector X Position', 'End Effector Z Position')
            title("Plot of End Effector Position vs Time")
            xlabel('Time (s)')
            ylabel('End Effector Position (mm)')
            hold off
            
            %Plot of Tip position XZ Plane
            subplot(4, 1, 3)
            hold on
            plot(mpMatrix(:, 5), mpMatrix(:, 7), 'k-') %End Effector X Position vs Z Position
            plot([points(:, 1); points(1,1)], [points(:, 3); points(1,3)], 'm-') %Expected Triangle
            legend('XZ Trajectory', 'Triangle Vertices')
            title("Trajectory of End Effector on XZ Plane")
            xlabel('X (mm)')
            ylabel('Z (mm)')
            hold off
            
            %Plot of Tip position XY Plane
            subplot(4, 1, 4)
            hold on
            plot(mpMatrix(:, 5), mpMatrix(:, 6), 'k-') %End Effector X Position vs Y Position
            plot(points(:, 1), points(:, 2), 'm-') %Expected Triangle
            legend('XZ Trajectory', 'Triangle Vertices')
            title("Trajectory of End Effector on XY Plane")
            xlabel('X (mm)')
            ylabel('Y (mm)')
            ylim([-10 10])
            hold off
        end
        
    end
    
end