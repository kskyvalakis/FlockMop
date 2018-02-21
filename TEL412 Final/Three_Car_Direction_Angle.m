function [fdistancer,fdistanceg,fdistanceye,Angler,Angleg,Angleye] = Three_Car_Direction_Angle(cam,targetX1,targetX2,targetX3,targetY1,targetY2,targetY3,pixel1Tocm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HERE WE CALCULATE THE DISTANCE AND ANGLE OF CAR 1 AND 2 AND 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%We take snapshots until we detect all the stickers
while(1)
    
    Im=snapshot(cam);
    %We brake the image to its RGB elements
    r = Im(:, :, 1);             % red channel
    g = Im(:, :, 2);             % green channel
    b = Im(:, :, 3);             % blue channel
    
    %We define the redness of a pixel with a specific threshold
    redness = double(r) - max(double(g), double(b));
    redmask = redness > 80;
    %We add a morphological filter in order to get rid of the salt noise
    redmask = bwareaopen(redmask,200);
    
    
    %We define the blueness of a pixel with a specific threshold
    blueness = double(b) - max(double(g), double(r));
    bluemask = blueness > 10;
    %We add a morphological filter in order to get rid of the salt noise
    bluemask = bwareaopen(bluemask,80);
    %We connect the blue sticker parts if they are detected separated
    se1 = strel('square',20);
    bluemask = imclose(bluemask,se1);
    
    
    %We define the yellowness of a pixel with a specific threshold
    yellowness = (double(r)+double(g))/2-double(b);
    yellowmask = yellowness > 90;
    %We separate the yellow noise
    se1 = strel('square',2);
    yellowmask = imerode(yellowmask,se1);
    %We add a morphological filter in order to get rid of the salt noise
    yellowmask = bwareaopen(yellowmask,300);
    %We connect the yellow sticker parts if they are detected separated
    yellowmask= imclose(yellowmask,se1);
    
    
    %We define the greennes of a pixel with a specific threshold
    greenness = double(g) - max(double(r), double(b));
    greenmask = greenness > 20;
    greenmask = bwareaopen(greenmask,250);
    
    labeledImage = logical(redmask);
    measurementsr = regionprops(labeledImage, 'Centroid');
    [sr,~]=size(measurementsr);
    
    labeledImage = logical(greenmask);
    measurementsg = regionprops(labeledImage, 'Centroid');
    [sg,~]=size(measurementsg);
    
    labeledImage = logical(bluemask);
    measurementsb = regionprops(labeledImage, 'Centroid');
    [sb,~]=size(measurementsb);
    
    labeledImage = logical(yellowmask);
    measurementsy = regionprops(labeledImage, 'Centroid');
    [sy,~]=size(measurementsy);
    
    %     figure
    %     imshow(redmask);title('redmask');
    %
    %     figure
    %     imshow(bluemask);title('bluemask');
    %
    %     figure
    %     imshow(greenmask);title('greenmask');
    %
    %     figure
    %     imshow(yellowmask);title('yellowmask');
    
    if(sr==1)&& (sg==1) && (sb==3) && (sy==1)
        rCenter = measurementsr.Centroid; % x,y coordinates
        gCenter = measurementsg.Centroid; % x,y coordinates
        yeCenter = measurementsy.Centroid; % x,y coordinates
        b1Center = measurementsb(1).Centroid; % x,y coordinates blue 1
        b2Center = measurementsb(2).Centroid; % x,y coordinates blue 2
        b3Center = measurementsb(3).Centroid; % x,y coordinates blue 2
        break;
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND WITCH BLUE BOX CONECTS WITH ITS RESPECTIVE ID BOX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tmp_dist1=sqrt((b1Center(1)-rCenter(1))^2+(b1Center(2)-rCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-rCenter(1))^2+(b2Center(2)-rCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-rCenter(1))^2+(b3Center(2)-rCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3];

[~,i]=min(tmp_dist);

if(i==1)
    brCenter=b1Center;
elseif(i==2)
    brCenter=b2Center;
else
    brCenter=b3Center;
end

tmp_dist1=sqrt((b1Center(1)-gCenter(1))^2+(b1Center(2)-gCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-gCenter(1))^2+(b2Center(2)-gCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-gCenter(1))^2+(b3Center(2)-gCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3];

[~,i]=min(tmp_dist);

if(i==1)
    bgCenter=b1Center;
elseif(i==2)
    bgCenter=b2Center;
else
    bgCenter=b3Center;
end

tmp_dist1=sqrt((b1Center(1)-yeCenter(1))^2+(b1Center(2)-yeCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-yeCenter(1))^2+(b2Center(2)-yeCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-yeCenter(1))^2+(b3Center(2)-yeCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3];

[~,i]=min(tmp_dist);

if(i==1)
    byeCenter=b1Center;
elseif(i==2)
    byeCenter=b2Center;
else
    byeCenter=b3Center;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE WITCH CAR IS CLOSEST TO A SPOT AND THEN MAKE IT TO GO THERE AND
% THE OTHER ONE TO THE OTHER SPOT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance1r=sqrt(((targetX1-rCenter(1))^2)+((targetY1-rCenter(2))^2))*pixel1Tocm; % the distance of red to go to one spot
distance2r=sqrt(((targetX2-rCenter(1))^2)+((targetY2-rCenter(2))^2))*pixel1Tocm; % the distance of red to go to the other
distance3r=sqrt(((targetX3-rCenter(1))^2)+((targetY3-rCenter(2))^2))*pixel1Tocm; % the distance of red to go to the other

distancer=[distance1r distance2r distance3r];

distance1g=sqrt(((targetX1-gCenter(1))^2)+((targetY1-gCenter(2))^2))*pixel1Tocm; % the distance of green to go to one spot
distance2g=sqrt(((targetX2-gCenter(1))^2)+((targetY2-gCenter(2))^2))*pixel1Tocm; % the distance of green to go to the other
distance3g=sqrt(((targetX3-gCenter(1))^2)+((targetY3-gCenter(2))^2))*pixel1Tocm; % the distance of green to go to the other

distanceg=[distance1g distance2g distance3g];

distance1ye=sqrt(((targetX1-yeCenter(1))^2)+((targetY1-yeCenter(2))^2))*pixel1Tocm; % the distance of green to go to one spot
distance2ye=sqrt(((targetX2-yeCenter(1))^2)+((targetY2-yeCenter(2))^2))*pixel1Tocm; % the distance of green to go to the other
distance3ye=sqrt(((targetX3-yeCenter(1))^2)+((targetY3-yeCenter(2))^2))*pixel1Tocm; % the distance of green to go to the other

distanceye=[distance1ye distance2ye distance3ye];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND THE  SPOT FOR EACH CAR WITH THE MINIMUM DISTANCE RULE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (min(distancer)<min(distanceg))&&(min(distancer)<min(distanceye))
    
    [fdistancer,i]=min(distancer);
    
    if(i==1)
        
        Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
        
        if(min([distanceg(2) distanceg(3)])<min([distanceye(2) distanceye(3)]))
            
            [fdistanceg,j]=min([distanceg(2) distanceg(3)]);
            
            
            if(j==1)
                fdistanceye=distanceye(3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
                
            else
                fdistanceye=distanceye(2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
                
            end
        else
            [fdistanceye,j]=min([distanceye(2) distanceye(3)]);
            
            if(j==1)
                fdistanceg=distanceg(3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            else
                fdistanceg=distanceg(2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            end
        end
        
        
    elseif(i==2)
        
        Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
        
        if(min([distanceg(1) distanceg(3)])<min([distanceye(1) distanceye(3)]))
            
            [fdistanceg,j]=min([distanceg(1) distanceg(3)]);
            
            if(j==1)
                fdistanceye=distanceye(3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            else
                fdistanceye=distanceye(1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            end
        else
            [fdistanceye,j]=min([distanceye(1) distanceye(3)]);
            
            if(j==1)
                fdistanceg=distanceg(3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            else
                fdistanceg=distanceg(1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            end
        end
    else
        Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
        
        if(min([distanceg(1) distanceg(2)])<min([distanceye(1) distanceye(2)]))
            [fdistanceg,j]=min([distanceg(1) distanceg(2)]);
            
            
            if(j==1)
                fdistanceye=distanceye(2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            else
                fdistanceye=distanceye(1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            end
        else
            [fdistanceye,j]=min([distanceye(1) distanceye(2)]);
            
            if(j==1)
                fdistanceg=distanceg(2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            else
                fdistanceg=distanceg(1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            end
        end
    end
elseif(min(distanceg)<min(distancer))&&(min(distanceg)<min(distanceye))
    [fdistanceg,i]=min(distanceg);
    
    if(i==1)
        Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
        if(min([distancer(2) distancer(3)])<min([distanceye(2) distanceye(3)]))
            [fdistancer,j]=min([distancer(2) distancer(3)]);
            
            if(j==1)
                fdistanceye=distanceye(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            else
                fdistanceye=distanceye(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            end
        else
            [fdistanceye,j]=min([distanceye(2) distanceye(3)]);
            
            if(j==1)
                fdistancer=distancer(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            else
                fdistancer=distancer(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            end
        end
    elseif(i==2)
        
        Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
        
        if(min([distancer(1) distancer(3)])<min([distanceye(1) distanceye(3)]))
            
            [fdistancer,j]=min([distancer(1) distancer(3)]);
            
            if(j==1)
                fdistanceye=distanceye(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            else
                fdistanceye=distanceye(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            end
        else
            [fdistanceye,j]=min([distanceye(1) distanceye(3)]);
            
            if(j==1)
                fdistancer=distancer(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            else
                fdistancer=distancer(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
            end
        end
    else
        
        Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
        
        if(min([distancer(1) distancer(2)])<min([distanceye(1) distanceye(2)]))
            [fdistancer,j]=min([distancer(1) distancer(2)]);
            
            if(j==1)
                fdistanceye=distanceye(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            else
                fdistanceye=distanceye(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            end
        else
            [fdistanceye,j]=min([distanceye(1) distanceye(2)]);
            
            if(j==1)
                fdistancer=distancer(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
            else
                fdistancer=distancer(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
            end
        end
    end
elseif(min(distanceye)<min(distanceg))&&(min(distanceye)<min(distancer))
    [fdistanceye,i]=min(distanceye);
    
    if(i==1)
        
        Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
        
        if(min([distancer(2) distancer(3)])<min([distanceg(2) distanceg(3)]))
            [fdistancer,j]=min([distancer(2) distancer(3)]);
            
            if(j==1)
                fdistanceg=distanceg(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
            else
                fdistanceg=distanceg(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
            end
        else
            [fdistanceg,j]=min([distanceg(2) distanceg(3)]);
            
            if(j==1)
                fdistancer=distancer(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
            else
                fdistancer=distancer(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
            end
        end
    elseif(i==2)
        
        Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
        
        if(min([distancer(1) distancer(3)])<min([distanceg(1) distanceg(3)]))
            [fdistancer,j]=min([distancer(1) distancer(3)]);
            
            if(j==1)
                fdistanceg=distanceg(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
            else
                fdistanceg=distanceg(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
            end
        else
            [fdistanceg,j]=min([distanceg(1) distanceg(3)]);
            
            if(j==1)
                fdistancer=distancer(3);
                Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
            else
                fdistancer=distancer(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
            end
        end
    else
        
        Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
        
        if(min([distancer(1) distancer(2)])<min([distanceg(1) distanceg(2)]))
            [fdistancer,j]=min([distancer(1) distancer(2)]);
            
            if(j==1)
                fdistanceg=distanceg(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
            else
                fdistanceg=distanceg(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
            end
        else
            [fdistanceg,j]=min([distanceg(1) distanceg(2)]);
            
            if(j==1)
                fdistancer=distancer(2);
                Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
            else
                fdistancer=distancer(1);
                Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
                Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
            end
        end
    end
end

end

