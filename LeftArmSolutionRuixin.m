%calculate the inverse kinematics for Nao's left arm
function [R] = LeftArmSolutionRuixin(target)

shoulderOffsetY = 98;
elbowOffsetY = 15;
upperArmLength = 105;
shoulderOffsetZ = 100;
HandOffsetX = 57.75;
HandOffsetZ = 12.31;
LowerArmLength = 55.95;
HipOffsetZ = 85;
HipOffsetY = 50;
ThighLength = 100;
TibiaLength = 102.90;
FootHeight = 45.11;
NeckOffsetZ = 126.5;

%save all thetas with a list 'result'
result = [];

a1 = target(4);
a2 = target(5);
a3 = target(6);
x = target(1) ;
y = target(2) ;
z = target(3) ;

T = RotZYXMatrix(a3,a2,a1);
T(1,4) = x;
T(2,4) = y;
T(3,4) = z;


base = eye(4,4);
base(2,4) = shoulderOffsetY;
base(3,4) = shoulderOffsetZ;

R = RotXYZMatrix(-pi/2,0,-pi/2);

Tend1 = eye(4,4);
Tend1(1,4) = HandOffsetX + LowerArmLength;
Tend1(3,4) = -HandOffsetZ;
Tend = R*Tend1;

l2 = elbowOffsetY;
l1 = upperArmLength;

Ti1 = base\T/Tend;
theta1 = [atan2(-Ti1(3,4),Ti1(1,4)) atan2(-Ti1(3,4),Ti1(1,4))];
T1 = DH(0,-pi/2,0,theta1(1));
Ti2 = T1\Ti1;
theta2temp = acos((l2*Ti2(1,4)-l1*Ti2(3,4))/(l1^2+l2^2));

LShoulderRollHigh = 1.3265;
LShoulderRollLow = -0.3142;
for i = 0:1:2
    if i == 0 && (theta2temp-pi/2 > LShoulderRollHigh || theta2temp-pi/2 < LShoulderRollLow)
        continue;
    end
    if i == 1 && (-theta2temp-pi/2 > LShoulderRollHigh || -theta2temp-pi/2 < LShoulderRollLow)
        continue;
    end
    if i == 1
        theta2temp = -theta2temp;
    end
    theta2 = theta2temp-pi/2;
end

T2 = DH(0,pi/2,0,theta2+pi/2);
Ti3 = T2\Ti2;
theta3 = [atan2(Ti3(3,3),Ti3(1,3)) atan2(Ti3(3,3),Ti3(1,3))];
T3 = DH(15,pi/2,upperArmLength,theta3(1));
Ti4 = T3\Ti3;
theta4 = [atan2(Ti4(1,3),Ti4(3,3)) atan2(Ti4(1,3),Ti4(3,3))];
theta5 = [atan2(Ti4(2,1),Ti4(2,2)) atan2(Ti4(2,1),Ti4(2,2))];
T4 = DH(0,-pi/2,0,theta4(1));
T5 = DH(0,pi/2,LowerArmLength,theta5(1));
thetas = [theta1(1),theta2,theta3(1),theta4(1),theta5(1)];
R = thetas;
result = [result;thetas];
%%visualization
%Visual(thetas); drawnow; hold on;