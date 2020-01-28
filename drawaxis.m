% draw X,Y,Z axis for a point
function drawaxis(Matrix)
x = Matrix(1:3,1);
y = Matrix(1:3,2);
z = Matrix(1:3,3);
l = 20;
plot3([Matrix(1,4),Matrix(1,4)+x(1)*l],[Matrix(2,4),Matrix(2,4)+x(2)*l],[Matrix(3,4),Matrix(3,4)+x(3)*l],'r');hold on;
plot3([Matrix(1,4),Matrix(1,4)+y(1)*l],[Matrix(2,4),Matrix(2,4)+y(2)*l],[Matrix(3,4),Matrix(3,4)+y(3)*l],'g');hold on;
plot3([Matrix(1,4),Matrix(1,4)+z(1)*l],[Matrix(2,4),Matrix(2,4)+z(2)*l],[Matrix(3,4),Matrix(3,4)+z(3)*l],'b');hold on;