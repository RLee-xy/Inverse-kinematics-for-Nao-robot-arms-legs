%This function is used to get a set of joint angles 'thetas' which can make
%the end effector of robot arm to reach the target point in high accuracy
function R = approximation(n,thetas_last)
% target = [87.36,282.97,90.481,pi,pi,pi];
% thetas = LeftArmSolutionRuixin(target);
%n = [187,67,-30];
g = 0;
R = [];
result = thetas_last;
result0 = [];
% for t = 0: pi/10:2*pi
%     target = [180 + 10*sin(t),90,120+10*cos(t)];
target = n;
theta = [];
s = 0;
distance = 10000;
for i = 0:pi/10:pi
    for j = 0:pi/10:pi
        for k = 0:pi/10:pi %try all the direction to reach the target
            targets = [target(1),target(2),target(3),i,j,k];
            try
                thetas = LeftArmSolutionRuixin(targets);
            catch
                s = s + 1;
                continue;
            end
            [Tend left] = fLeftHandH25(thetas);
            dis = (left(1)-target(1))^2 + (left(2)-target(2))^2 + (left(2)-target(2))^2 ;
            if dis < 2
                theta = [theta;thetas];%store all 'thetas' that satisify the requirement for high accuracy
            end
        end
    end
end

%find a set of 'thetas' that is the closest to the last set of joint
%angles
for m = 1:1:size(theta,1)
    theta1 = theta(m,:);
    diss = (theta1(1) - result(1))^2 + (theta1(2) - result(2))^2 + (theta1(3) - result(3))^2 + (theta1(4) - result(4))^2 + 5 * (theta1(5) - result(5))^2;
    if diss < distance
        distance = diss;
        result0 = theta1;
    end
    
end
result = result0;
R = result;
%   Visual(result);
%     Visual(theta);
%         g = g+1;
% end

