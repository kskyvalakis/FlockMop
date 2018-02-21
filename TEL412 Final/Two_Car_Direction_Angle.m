function [fdistancer,fdistanceg,Angler,Angleg] = Two_Car_Direction_Angle(cam,targetX1,targetX2,targetY1,targetY2,pixel1Tocm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HERE WE CALCULATE THE DISTANCE AND ANGLE OF CAR 1 AND 2
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
    
    %     figure
    %     imshow(redmask)
    
    %We define the greennes of a pixel with a specific threshold
    greenness = double(g) - max(double(r), double(b));
    greenmask = greenness > 20;
    greenmask = bwareaopen(greenmask,250)
    
    %     figure
    %     imshow(greenmask)
    
    
    %     figure
    %     imshow(bluemask)
    
    labeledImage = logical(redmask);
    measurementsr = regionprops(labeledImage, 'Centroid');
    [sr,~]=size(measurementsr);
    
    labeledImage = logical(greenmask);
    measurementsg = regionprops(labeledImage, 'Centroid');
    [sg,~]=size(measurementsg);
    
    labeledImage = logical(bluemask);
    measurementsb = regionprops(labeledImage, 'Centroid');
    [sb,~]=size(measurementsb);
    
    if(sr==1)&& (sg==1) && (sb==2)
        rCenter = measurementsr.Centroid; % x,y coordinates
        gCenter = measurementsg.Centroid; % x,y coordinates
        b1Center = measurementsb(1).Centroid; % x,y coordinates blue 1
        b2Center = measurementsb(2).Centroid; % x,y coordinates blue 2
        break;
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND WITCH BLUE BOX CONECTS WITH ITS RESPECTIVE ID BOX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dist_rb1=sqrt((b1Center(1)-rCenter(1))^2+(b1Center(2)-rCenter(2))^2);
dist_rb2=sqrt((b2Center(1)-rCenter(1))^2+(b2Center(2)-rCenter(2))^2);

if(dist_rb1<dist_rb2)
    brCenter=b1Center;
    bgCenter=b2Center;
else
    brCenter=b2Center;
    bgCenter=b1Center;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE WITCH CAR IS CLOSEST TO A SPOT AND THEN MAKE IT TO GO THERE AND
% THE OTHER ONE TO THE OTHER SPOT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance1r=sqrt(((targetX1-rCenter(1))^2)+((targetY1-rCenter(2))^2))*pixel1Tocm; % the distance of red to go to one spot
distance2r=sqrt(((targetX2-rCenter(1))^2)+((targetY2-rCenter(2))^2))*pixel1Tocm; % the distance of red to go to the other

distancer=[distance1r distance2r];

distance1g=sqrt(((targetX1-gCenter(1))^2)+((targetY1-gCenter(2))^2))*pixel1Tocm; % the distance of green to go to one spot
distance2g=sqrt(((targetX2-gCenter(1))^2)+((targetY2-gCenter(2))^2))*pixel1Tocm; % the distance of green to go to the other

distanceg=[distance1g distance2g];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND THE CLOSEST SPOT FOR EACH CAR WITH THE MINIMUM DISTANCE RULE 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IF THE RED IS THE CLOSEST TO THE SPOT
if min(distancer)<min(distanceg)
    
    [fdistancer,i]=min(distancer);
    
    fdistancer=ceil(fdistancer);
    
    if(i==1)
        fdistanceg=ceil(distanceg(2));
        figure(5)
        imshow(Im)
        hold on;
        plot(targetX1,targetY1,'ro',targetX2,targetY2,'go')
        hold off;
    elseif(i==2)
        fdistanceg=ceil(distanceg(1));
        figure(5)
        imshow(Im)
        hold on;
        plot(targetX2,targetY2,'ro',targetX1,targetY1,'go')
        hold off;
    end
    
    if(i==1)
        Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
    else
        Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
    end
    
    if(i==2)
        Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
        
    else
        Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
        
    end
    
else
    
    [fdistanceg,i]=min(distanceg);
    
    
    fdistanceg=ceil(fdistanceg);
    
    if(i==1)
        fdistancer=ceil(distancer(2));
        figure(5)
        imshow(Im)
        hold on;
        plot(targetX1,targetY1,'go',targetX2,targetY2,'ro')
        hold on;
    elseif(i==2)
        fdistancer=ceil(distancer(1));
        figure(5)
        imshow(Im)
        hold on;
        plot(targetX2,targetY2,'go',targetX1,targetY1,'ro')
        hold on;
    end
    
    if(i==1)
        Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
        
    else
        Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
    end
    
    if(i==2)
        Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
    else
        Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
    end
    
end

end

