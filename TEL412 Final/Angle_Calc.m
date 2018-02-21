%TRANSLATION ROTATION
%In order to find the angle that the car must rotate
function [angle] = Angle_Calc(bC,iC,tX,tY)
%Translation from image plane system to the cartesian plane
DirVector1=[floor(bC(1)),floor(bC(2))]-[floor(iC(1)),floor(iC(2))];
DirVector1(2)= -DirVector1(2);
DirVector2=[tX,tY]-[floor(iC(1)),floor(iC(2))];
DirVector2(2)= -DirVector2(2);

%Reference  vector in order to rotate the car direction vector to the y axis
referationVect=[0 1];

%If the car x coordinate is positive the rotation is counter-clockwise else its clockwise
if (DirVector1(1)>0)
    rangle=rad2deg(acos(dot(DirVector1,referationVect)/norm(DirVector1)/norm(referationVect))); %rotation angle
else
    rangle=-rad2deg(acos(dot(DirVector1,referationVect)/norm(DirVector1)/norm(referationVect))); %rotation angle
end

%We rotate the direction vector 2 with the same angle
xgrot=DirVector2(1)*cosd(rangle)-DirVector2(2)*sind(rangle);

%We calculate the angle between the 2 rotated direction vectors
angle=ceil(rad2deg(acos(dot(DirVector1,DirVector2)/norm(DirVector1)/norm(DirVector2))));

% if x of the second rotated direction vector is possitive we have a right
% rotation else we have a left rotation
% We define the negative angle in a way to be compatible with the cars
% instuctions (0-360 angle)
if (xgrot<0)
    disp('turn left')
else
    angle=360-angle;
    disp('turn right')
end

end

