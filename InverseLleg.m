function R = InverseLlge(target)

HipOffsetZ = 85; 
HipOffsetY = 50;
ThighLength = 100;
TibiaLength = 102.90;
FootHeight = 45.11;

%initialize
base = eye(4,4);
base(2,4) = HipOffsetY;
base(3,4) = -HipOffsetZ;

% test case, calculate the target T
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

R = RotZYXMatrix(pi,-pi/2,0);
Tend1 = eye(4,4);
Tend1(3,4) = -FootHeight;
Tend = R;
%forward kinematics Tendend = base*T1*T2*T3*T4*T5*T6*Tend*Tend1;
%inverse Kinematic
Tia = base\T/Tend1;
Tia = RotZYXMatrix(0,0,pi/4) * Tia;
Ti0 = inv(Tia);

%caculate thetas

l1 = ThighLength;
l2 = TibiaLength;
flag = 1;
d = Ti0(1,4)^2 + Ti0(2,4)^2 + Ti0(3,4)^2;
%theta4 = [pi - acos((l1^2 + l2^2 + d)/(2*l1*l2)),-pi+acos((l1^2 + l2^2 + d)/(2*l1*l2))];
theta4 = pi - acos((l1^2 + l2^2 - d)/(2*l1*l2));
%thetax = theta4;
theta6 = atan2(Ti0(2,4),Ti0(3,4));

% range of theta6
LAnkleRollLow = -0.3978;
LAnkleRollHigh = 0.7690;

% if theta6 in the correct range
if theta6 > LAnkleRollHigh || theta6 < LAnkleRollLow
    flag = 0;
end
T6 = DH(0,-pi/2,0,theta6);
Ti10 = Tia/(T6*R);
Ti1 = inv(Ti10);

%range of theta1 to theta5
RKneePitchHigh = 2.1201;
RKneePitchLow = -0.1030;

LAnklePitchHigh = 0.9227;
LAnklePitchLow = -1.1895;

LHipRollHigh = 0.7904;
LHipRollLow	 = -0.3794;

LHipPitchHigh = 0.4840;
LHipPitchLow = -1.7739;

LHipYawPitchHigh = 0.7408;
LHipYawPitchLow = -1.1453;

%get the right solution for thetas according to range of theta1 to theta5
if flag == 1
    for i = 0:1:1 %calculate theta4
        if i ~= 0
            theta4 = -theta4;
        end
        if theta4 > RKneePitchHigh || theta4 < RKneePitchLow
            if i == 1
                theta4 = -theta4;
            end
            continue;
        end
        % set up DH homogeneous matrix for theta4 and calculate theta5
        T4 = DH(-ThighLength,0,0,theta4);
        theta5 = asin(-(Ti1(2,4)*(l2+l1*cos(theta4))+ l1*Ti1(1,4)*sin(theta4))/(l1^2*sin(theta4)^2+(l2+l1*cos(theta4))^2));
        if theta5 >= 0
            k = pi;
        else
            k = -pi;
        end
         if theta5 ~= theta5 && (Ti1(2,4)*(l2+l1*cos(theta4))+ l1*Ti1(1,4)*sin(theta4))/(l1^2*sin(theta4)^2+(l2+l1*cos(theta4))^2) < 0
 			theta5 = -pi/2;
         elseif theta5 ~= theta5
 			theta5 = pi/2;
         end 
        for j = 0:1:1 %calculate theta5
            if j == 0 && (theta5 > LAnklePitchHigh || theta5 < LAnklePitchLow)
                continue;
            elseif j == 1 && (k - theta5 > LAnklePitchHigh || k - theta5 < LAnklePitchLow)
                continue;
            elseif j == 1
                theta5 = k - theta5;
            end
            T5 = DH(-TibiaLength,0,0,theta5);

            Ti2 = Ti10/(T4*T5);
            theta20 = acos(Ti2(2,3));
            theta2 = 0;
            for s = 0:1:1 %calculate theta2
               if s == 0 && (theta20 - pi/4 > LHipRollHigh || theta20 - pi/4 < LHipRollLow)
                   continue;
               elseif s == 1 && (-theta20 - pi/4 > LHipRollHigh || -theta20 - pi/4 < LHipRollLow)
                   continue;
               elseif s == 0
                   theta2 = theta20-pi/4;
               elseif s == 1
                   theta2 = -theta20-pi/4;
               end
               theta3 = asin(Ti2(2,2)/sin(theta2+pi/4));
               if theta3 >= 0
                   t = pi;
               else
                   t = -pi;
               end
                if theta3 ~= theta3 && Ti2(2,2)/sin(theta2+pi/4) < 0
 					theta3 = -pi/2;
                elseif theta3 ~= theta3
 					theta3 = pi/2;
                end
               for l = 0:1:1 %calculate theta3
                   if l == 0 && (theta3 > LHipPitchHigh || theta3 < LHipPitchLow)
                       continue;
                   elseif l == 1 && (t - theta3 > LHipPitchHigh || t - theta3 < LHipPitchLow)
                       continue;
                   elseif l == 1
                       theta3 = t - theta3;
                   end
                   theta10 = acos(Ti2(1,3)/sin(theta2+pi/4));
                   if theta10 ~= theta10
                       theta10 = 0;
                   end
                   for p = 0:1:1 %calculate theta1
                       if p == 0 && (theta10 + pi/2 > LHipYawPitchHigh || theta10 + pi/2 < LHipYawPitchLow)
                           continue;
                       elseif p == 1 && (-theta10 + pi/2 > LHipYawPitchHigh || -theta10 + pi/2 < LHipYawPitchLow)
                           continue;
                       elseif p == 0
                           theta1 = theta10 + pi/2;
                       elseif p == 1
                           theta1 = -theta10 + pi/2;
                       end
                       
                   end
                   
               end
               
            end
            
        end
    end
end
R = [theta1,theta2,theta3,theta4,theta5,theta6]
%%visualization
%Visual_leg(R); drawnow; hold on;