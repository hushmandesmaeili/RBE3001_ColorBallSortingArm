%File to test the Frame3D class

close all

size = 20;
T = [0, -1, 0, 100;
     1, 0, 0, 0;
     0, 0, 1, 195;
     0, 0, 0, 1];
 
T00 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];

frame = Frame3D();

plot3(T(1, 4), T(2, 4), T(3, 4), '.')
hold on
grid on;
xlim([-300 300]);
ylim([-300 300]);
zlim([-300 300]);
xlabel('x position')
ylabel('y position')
zlabel('z position')

frame.showFrame(T00, size);
frame.showFrame(T, size);



% Initial frames test, created a class after this

% % X-axis, red
% quiver3(T(1, 4), T(2, 4), T(3, 4), size*T(1, 1), size*T(2, 1), size*T(3, 1), 'LineWidth', 2, 'Color', 'r')
% 
% % Y-axis, green
% quiver3(T(1, 4), T(2, 4), T(3, 4), size*T(1, 2), size*T(2, 2), size*T(3, 2), 'LineWidth', 2, 'Color', 'g')
% 
% % Z-axis, blue
% quiver3(T(1, 4), T(2, 4), T(3, 4), size*T(1, 3), size*T(2, 3), size*T(3, 3), 'LineWidth', 2, 'Color', 'b')