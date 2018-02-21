function [fdistancer,fdistanceg,fdistanceye,fdistancecy,Angler,Angleg,Angleye,Anglecy] = Four_Car_Direction_Angle(cam,targetX1,targetX2,targetX3,targetX4,targetY1,targetY2,targetY3,targetY4,pixel1Tocm,whitemask)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HERE WE CALCULATE THE DISTANCE AND ANGLE OF CAR 1 2 3 AND 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%We take snapshots until we detect all the stickers
while(1)
    
    %     figure
    %     imshow(Im)
    Im=snapshot(cam);
    %We brake the image to its RGB elements
    r = Im(:, :, 1);             % red channel
    g = Im(:, :, 2);             % green channel
    b = Im(:, :, 3);             % blue channel
    
    %We define the redness of a pixel with a specific threshold
    redness = double(r) - max(double(g), double(b));
    redmaskbf = redness > 80;
    %We add a morphological filter in order to get rid of the salt noise
    redmask = bwareaopen(redmaskbf,200);
    
    
    %We define the blueness of a pixel with a specific threshold
    blueness = double(b) - max(double(g), double(r));
    bluemaskbf = blueness > 10;
    %We add a morphological filter in order to get rid of the salt noise
    bluemask = bwareaopen(bluemaskbf,80);
    %We connect the blue sticker parts if they are detected separated
    se1 = strel('square',20);
    bluemask = imclose(bluemask,se1);
    
    %We define the yellowness of a pixel with a specific threshold
    yellowness = (double(r)+double(g))/2-double(b);
    yellowmaskbf = yellowness > 90;
    %We separate the yellow noise
    se1 = strel('square',2);
    yellowmask = imerode(yellowmaskbf,se1);
    %We add a morphological filter in order to get rid of the salt noise
    yellowmask = bwareaopen(yellowmask,300);
    %We connect the yellow sticker parts if they are detected separated
    yellowmask= imclose(yellowmask,se1);
    
    %We define the cyanness of a pixel with a specific threshold
    cyannmaskbf = ((double(b)>150).*(double(r)>120).*(double(g)>180))-whitemask;
    %We add a morphological filter in order to get rid of the salt noise
    cyannmask = bwareaopen(cyannmaskbf,250);
    
    %We define the greennes of a pixel with a specific threshold
    greenness = double(g) - max(double(r), double(b));
    greenmaskbf = greenness > 20;
    greenmask = bwareaopen(greenmaskbf,250);
    
    %We find the centers of each sticker
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
    
    labeledImage = logical(cyannmask);
    measurementscy = regionprops(labeledImage, 'Centroid');
    [scy,~]=size(measurementscy);
    
    %Only if we detect all the colored stickers we proceed with the routing
    %algorithm
    if(sr==1)&& (sg==1) && (sb==4) && (sy==1) && (scy==1)
        
%         figure
%         subplot(2,1,1)
%         imshow(redmaskbf);title('redmask filtering');ylabel('Before');
%         subplot(2,1,2)
%         imshow(redmask);ylabel('After');
%         figure
%         subplot(2,1,1)
%         imshow(greenmaskbf);title('greenmask filtering');ylabel('Before');
%         subplot(2,1,2)
%         imshow(greenmask);ylabel('After');
%         figure
%         subplot(2,1,1)
%         imshow(yellowmaskbf);title('yellowmask filtering');ylabel('Before');
%         subplot(2,1,2)
%         imshow(yellowmask);ylabel('After');
%         figure
%         subplot(2,1,1)
%         imshow(cyannmaskbf);title('cyannmask filtering');ylabel('Before');
%         subplot(2,1,2)
%         imshow(cyannmask);ylabel('After');
%         figure
%         subplot(2,1,1)
%         imshow(bluemaskbf);title('bluemask filtering');ylabel('Before');
%         subplot(2,1,2)
%         imshow(bluemask);ylabel('After');
        
        rCenter = measurementsr.Centroid; % x,y coordinates red car
        gCenter = measurementsg.Centroid; % x,y coordinates green car
        yeCenter = measurementsy.Centroid; % x,y coordinates yellow car
        cyCenter = measurementscy.Centroid; % x,y coordinates cyann car
        b1Center = measurementsb(1).Centroid; % x,y coordinates blue 1
        b2Center = measurementsb(2).Centroid; % x,y coordinates blue 2
        b3Center = measurementsb(3).Centroid; % x,y coordinates blue 3
        b4Center = measurementsb(4).Centroid; % x,y coordinates blue 4
        figure
        imshow(Im)
        hold on;
        plot(targetX1,targetY1,'bo',targetX2,targetY2,'bo',targetX3,targetY3,'bo',targetX4,targetY4,'bo')
        hold off;
        break;
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND WHICH BLUE BOX CONNECTS WITH ITS RESPECTIVE ID BOX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tmp_dist1=sqrt((b1Center(1)-rCenter(1))^2+(b1Center(2)-rCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-rCenter(1))^2+(b2Center(2)-rCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-rCenter(1))^2+(b3Center(2)-rCenter(2))^2);
tmp_dist4=sqrt((b4Center(1)-rCenter(1))^2+(b4Center(2)-rCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3 tmp_dist4];

[~,i]=min(tmp_dist);

if(i==1)
    brCenter=b1Center;
elseif(i==2)
    brCenter=b2Center;
elseif(i==3)
    brCenter=b3Center;
else
    brCenter=b4Center;
end

tmp_dist1=sqrt((b1Center(1)-gCenter(1))^2+(b1Center(2)-gCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-gCenter(1))^2+(b2Center(2)-gCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-gCenter(1))^2+(b3Center(2)-gCenter(2))^2);
tmp_dist4=sqrt((b4Center(1)-gCenter(1))^2+(b4Center(2)-gCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3 tmp_dist4];

[~,i]=min(tmp_dist);

if(i==1)
    bgCenter=b1Center;
elseif(i==2)
    bgCenter=b2Center;
elseif(i==3)
    bgCenter=b3Center;
else
    bgCenter=b4Center;
end

tmp_dist1=sqrt((b1Center(1)-yeCenter(1))^2+(b1Center(2)-yeCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-yeCenter(1))^2+(b2Center(2)-yeCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-yeCenter(1))^2+(b3Center(2)-yeCenter(2))^2);
tmp_dist4=sqrt((b4Center(1)-yeCenter(1))^2+(b4Center(2)-yeCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3 tmp_dist4];

[~,i]=min(tmp_dist);

if(i==1)
    byeCenter=b1Center;
elseif(i==2)
    byeCenter=b2Center;
elseif(i==3)
    byeCenter=b3Center;
else
    byeCenter=b4Center;
end

tmp_dist1=sqrt((b1Center(1)-cyCenter(1))^2+(b1Center(2)-cyCenter(2))^2);
tmp_dist2=sqrt((b2Center(1)-cyCenter(1))^2+(b2Center(2)-cyCenter(2))^2);
tmp_dist3=sqrt((b3Center(1)-cyCenter(1))^2+(b3Center(2)-cyCenter(2))^2);
tmp_dist4=sqrt((b4Center(1)-cyCenter(1))^2+(b4Center(2)-cyCenter(2))^2);

tmp_dist=[tmp_dist1 tmp_dist2 tmp_dist3 tmp_dist4];

[~,i]=min(tmp_dist);

if(i==1)
    bmagCenter=b1Center;
elseif(i==2)
    bmagCenter=b2Center;
elseif(i==3)
    bmagCenter=b3Center;
else
    bmagCenter=b4Center;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Car Routing Algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
car_coor(1,1)=rCenter(1);
car_coor(2,1)=gCenter(1);
car_coor(3,1)=yeCenter(1);
car_coor(4,1)=cyCenter(1);
car_coor(1,2)=rCenter(2);
car_coor(2,2)=gCenter(2);
car_coor(3,2)=yeCenter(2);
car_coor(4,2)=cyCenter(2);

%4 targets Coordinates

target_coor(1,1)=targetX1;
target_coor(2,1)=targetX2;
target_coor(3,1)=targetX3;
target_coor(4,1)=targetX4;
target_coor(1,2)=targetY1;
target_coor(2,2)=targetY2;
target_coor(3,2)=targetY3;
target_coor(4,2)=targetY4;

distance=zeros(4,4); %first column is car  and second column is target
matching=zeros(4,2);  %first column is car  and second column is target

%filling Distance matrix (between targets and cars)
for i=1:4
    for j=1:4
        distance(i,j)=sqrt((car_coor(i,1)-target_coor(j,1))^2+(car_coor(i,2)-target_coor(j,2))^2);
    end
end

for i=1:4
    
    [~,id]=max(reshape(distance',[16,1]));
    
    if rem(id,4)==0
        maxj=4;
    else
        maxj=rem(id,4);
    end
    %now that i have found the max distance target i will find
    % which car is closer to it and send it there
    
    column=distance(:,maxj);
    for j=1:4
        if column(j)==0
            column(j)=inf;
        end
    end
    
    [~,id_min]=min(column);
    
    matching(i,1)=id_min;
    matching(i,2)=maxj;
    
    distance(:,maxj)=0;
    distance(id_min,:)=0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND THE  SPOT FOR EACH CAR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:4
    if(matching(i,1)==1) % the car is the red car
        if(matching(i,2)==1) % the red car goes to spot 1
            fdistancer=sqrt(((targetX1-rCenter(1))^2)+((targetY1-rCenter(2))^2))*pixel1Tocm;
            Angler=Angle_Calc(brCenter,rCenter,targetX1,targetY1);
        elseif(matching(i,2)==2)% the red car goes to spot 2
            fdistancer=sqrt(((targetX2-rCenter(1))^2)+((targetY2-rCenter(2))^2))*pixel1Tocm;
            Angler=Angle_Calc(brCenter,rCenter,targetX2,targetY2);
        elseif(matching(i,2)==3)% the red car goes to spot 3
            fdistancer=sqrt(((targetX3-rCenter(1))^2)+((targetY3-rCenter(2))^2))*pixel1Tocm;
            Angler=Angle_Calc(brCenter,rCenter,targetX3,targetY3);
        else                   % the red car goes to spot 4
            fdistancer=sqrt(((targetX4-rCenter(1))^2)+((targetY4-rCenter(2))^2))*pixel1Tocm;
            Angler=Angle_Calc(brCenter,rCenter,targetX4,targetY4);
        end
    elseif(matching(i,1)==2) % the car is the green car
        if(matching(i,2)==1) % the green car goes to spot 1
            fdistanceg=sqrt(((targetX1-gCenter(1))^2)+((targetY1-gCenter(2))^2))*pixel1Tocm;
            Angleg=Angle_Calc(bgCenter,gCenter,targetX1,targetY1);
        elseif(matching(i,2)==2)% the green car goes to spot 2
            fdistanceg=sqrt(((targetX2-gCenter(1))^2)+((targetY2-gCenter(2))^2))*pixel1Tocm;
            Angleg=Angle_Calc(bgCenter,gCenter,targetX2,targetY2);
        elseif(matching(i,2)==3)% the green car goes to spot 3
            fdistanceg=sqrt(((targetX3-gCenter(1))^2)+((targetY3-gCenter(2))^2))*pixel1Tocm;
            Angleg=Angle_Calc(bgCenter,gCenter,targetX3,targetY3);
        else                   % the green car goes to spot 4
            fdistanceg=sqrt(((targetX4-gCenter(1))^2)+((targetY4-gCenter(2))^2))*pixel1Tocm;
            Angleg=Angle_Calc(bgCenter,gCenter,targetX4,targetY4);
        end
    elseif(matching(i,1)==3) % the car is the yellow car
        if(matching(i,2)==1) % the yellow car goes to spot 1
            fdistanceye=sqrt(((targetX1-yeCenter(1))^2)+((targetY1-yeCenter(2))^2))*pixel1Tocm;
            Angleye=Angle_Calc(byeCenter,yeCenter,targetX1,targetY1);
        elseif(matching(i,2)==2)% the yellow car goes to spot 2
            fdistanceye=sqrt(((targetX2-yeCenter(1))^2)+((targetY2-yeCenter(2))^2))*pixel1Tocm;
            Angleye=Angle_Calc(byeCenter,yeCenter,targetX2,targetY2);
        elseif(matching(i,2)==3)% the yellow car goes to spot 3
            fdistanceye=sqrt(((targetX3-yeCenter(1))^2)+((targetY3-yeCenter(2))^2))*pixel1Tocm;
            Angleye=Angle_Calc(byeCenter,yeCenter,targetX3,targetY3);
        else                   % the yellow car goes to spot 4
            fdistanceye=sqrt(((targetX4-yeCenter(1))^2)+((targetY4-yeCenter(2))^2))*pixel1Tocm;
            Angleye=Angle_Calc(byeCenter,yeCenter,targetX4,targetY4);
        end
    else                     % the car is the purple car
        if(matching(i,2)==1) % the purple car goes to spot 1
            fdistancecy=sqrt(((targetX1-cyCenter(1))^2)+((targetY1-cyCenter(2))^2))*pixel1Tocm;
            Anglecy=Angle_Calc(bmagCenter,cyCenter,targetX1,targetY1);
        elseif(matching(i,2)==2)% the purple car goes to spot 2
            fdistancecy=sqrt(((targetX2-cyCenter(1))^2)+((targetY2-cyCenter(2))^2))*pixel1Tocm;
            Anglecy=Angle_Calc(bmagCenter,cyCenter,targetX2,targetY2);
        elseif(matching(i,2)==3)% the purple car goes to spot 3
            fdistancecy=sqrt(((targetX3-cyCenter(1))^2)+((targetY3-cyCenter(2))^2))*pixel1Tocm;
            Anglecy=Angle_Calc(bmagCenter,cyCenter,targetX3,targetY3);
        else                   % the purple car goes to spot 4
            fdistancecy=sqrt(((targetX4-cyCenter(1))^2)+((targetY4-cyCenter(2))^2))*pixel1Tocm;
            Anglecy=Angle_Calc(bmagCenter,cyCenter,targetX4,targetY4);
        end
    end
    
end

end
