%visualization function for Nao's legs
function visual_leg(thetas)

HipOffsetZ = 85;
HipOffsetY = 50;
ThighLength = 100;
TibiaLength = 102.90;
FootHeight = 45.11;

% tset thetas
%thetas = [0,0,0,0,0,0];

%Forward Kinematics (DH)
base = eye(4,4);
base(2,4) = HipOffsetY;
base(3,4) = -HipOffsetZ;

T1 = DH(0,-3*pi/4,0,thetas(1)-pi/2);
T2 = DH(0,-pi/2,0,thetas(2)+pi/4);
T3 = DH(0,pi/2,0,thetas(3));
T4 = DH(-ThighLength,0,0,thetas(4));
T5 = DH(-TibiaLength,0,0,thetas(5));
T6 = DH(0,-pi/2,0,thetas(6));

R = RotZYXMatrix(pi,-pi/2,0);
Tend1 = eye(4,4);
Tend1(3,4) = -FootHeight;
Tend = R;

Tendend = base*T1*T2*T3*T4*T5*T6*Tend*Tend1;
THip = base*T1*T2*T3;
TKnee = base*T1*T2*T3*T4;
TAnkle = base*T1*T2*T3*T4*T5*T6;

zero = eye(4,4)
THip = THip - Tendend;
TKnee = TKnee - Tendend;
TAnkle = TAnkle - Tendend;
zero = zero - Tendend;
Tendend = Tendend - Tendend;

%visualize
plot3(0,0,0,'ro');hold on;
plot3(zero(1,4),zero(2,4),zero(3,4),'ro');hold on;
plot3(THip(1,4),THip(2,4),THip(3,4),'ro');hold on;
plot3(TKnee(1,4),TKnee(2,4),TKnee(3,4),'ro');hold on;
plot3(TAnkle(1,4),TAnkle(2,4),TAnkle(3,4),'ro');hold on;
plot3(Tendend(1,4),Tendend(2,4),Tendend(3,4),'ro');hold on;
%drawaxis(Tendend);
%plot3([-Tendend(1,4),THip(1,4)],[-Tendend(2,4),THip(2,4)],[-Tendend(3,4),THip(3,4)],'r');hold on;
plot3([zero(1,4),THip(1,4)],[zero(2,4),THip(2,4)],[zero(3,4),THip(3,4)],'r');hold on;
plot3([THip(1,4),TKnee(1,4)],[THip(2,4),TKnee(2,4)],[THip(3,4),TKnee(3,4)],'r');hold on;
plot3([TKnee(1,4),TAnkle(1,4)],[TKnee(2,4),TAnkle(2,4)],[TKnee(3,4),TAnkle(3,4)],'r');hold on;
plot3([TAnkle(1,4),Tendend(1,4)],[TAnkle(2,4),Tendend(2,4)],[TAnkle(3,4),Tendend(3,4)],'r');hold on;
xlabel('x');ylabel('y');zlabel('z');
xlim([-100,100]),ylim([-100,100]),zlim([-100,400]);axis equal;

% rotZ = atan2(Tendend(2,1),Tendend(1,1));
% rotY = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2 + Tendend(3,3)^2));
% rotX = atan2(Tendend(3,2),Tendend(3,3));
%
% str = sprintf('%f,%f,%f,%f,%f,%f',Tendend(1,4),Tendend(2,4),Tendend(3,4),rotX,rotY,rotZ);
% disp(str)
% left = [Tendend(1:3,4);rotX;rotY;rotZ];