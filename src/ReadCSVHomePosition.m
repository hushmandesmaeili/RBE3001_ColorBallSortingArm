close all;
T = readmatrix('Home_Position_10_trials.csv');

distance = zeros(1, 10);
ref = [100 0 195];

for n = 1:10
    
    % Plotting 
    grid on;
    plot3(T(n, 13), T(n, 14), T(n, 15), '.')
    xlim([99 102])
    ylim([-1 3])
    zlim([190 196])
    hold on
    
    % Computing 
    distance(1, n) = norm(ref - T(n, 13:15));
end

title('Home Position 10 Trials')
xlabel('x position')
ylabel('y position')
zlabel('z position')
meanX = mean(T(:, 13)) %prints mean x
meanY = mean(T(:, 14)) %prints mean y 
meanZ = mean(T(:, 15)) %prints mean z
plot3(meanX, meanY, meanZ, 'o');
plot3(ref(1), ref(2), ref(3), '*')
legend('1', '2', '3', '4', '5', '6', '7', '8', '9', '10', 'average', 'expected');
hold off

avg = mean(distance)
rms_val = rms(distance)

