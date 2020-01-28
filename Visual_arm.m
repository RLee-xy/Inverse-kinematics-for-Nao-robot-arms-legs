%visualization function for Nao's arms
function Visual(thetas)

%% Initializing the arm parameters
shoulderOffsetY = 98;
elbowOffsetY = 15;
upperArmLength = 105;
shoulderOffsetZ = 100;
HandOffsetX = 57.75;
HandOffsetZ = 12.31;
LowerArmLength = 55.95;

base = eye(4,4);
base(2,4) = shoulderOffsetY;
base(3,4) = shoulderOffsetZ;

%% Assign shoulder parameters
%thetas = [-45*pi/180, 45*pi/180,  -45*pi/180, -45*pi/180, 0]; % LShoulderP LShoulderY LElbowRoll LElbowYaw LWristRoll

%% Forward kinematics DH convention
T1 = DH(0,-pi/2,0,thetas(1));
T2 = DH(0,pi/2,0,thetas(2)+pi/2);
T3 = DH(elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = DH(0,-pi/2,0,thetas(4));
T5 = DH(0,pi/2,LowerArmLength,thetas(5));

TShoulder = base*T1*T2;
TElbow = base*T1*T2*T3*T4;
TWrist = base*T1*T2*T3*T4*T5;

Tend1 = eye(4,4);
Tend1(1,4) = HandOffsetX;
Tend1(3,4) = -HandOffsetZ;

R = RotXYZMatrix(-pi/2,0,-pi/2);
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*T5*Tend;

%% visualize result
plot3(TShoulder(1,4),TShoulder(2,4),TShoulder(3,4),'ro');hold on
plot3(TElbow(1,4),TElbow(2,4),TElbow(3,4),'ro');hold on
plot3(TWrist(1,4),TWrist(2,4),TWrist(3,4),'ro');hold on
plot3(Tendend(1,4),Tendend(2,4),Tendend(3,4),'ro');hold on;
% drawaxis(Tendend);
plot3([TShoulder(1,4),TElbow(1,4)],[TShoulder(2,4),TElbow(2,4)],[TShoulder(3,4),TElbow(3,4)],'r');hold on
plot3([TElbow(1,4),TWrist(1,4)],[TElbow(2,4),TWrist(2,4)],[TElbow(3,4),TWrist(3,4)],'r');hold on
plot3([TWrist(1,4),Tendend(1,4)],[TWrist(2,4),Tendend(2,4)],[TWrist(3,4),Tendend(3,4)],'r');hold on
xlabel('x');ylabel('y');zlabel('z');
xlim([-200,200]),ylim([-200,200]),zlim([-20,300]);axis equal
