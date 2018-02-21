%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors : Kyriakos Psarakis
%           Lefteris Tzagkarakis
%           Marios Vestakis
%           Konstantinos Skivalakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This project was part of the undergraduate course TEL 412 that took place
% in the Technical University of Crete School of Electronics and Computer
% Engineering. The goal of the project was to coordinate the movement of 2-4
% cars in the 2-D plane in order to create specific shapes. In order to
% achieve that goal we used a camera for the car localization, the Silicon
% Laboratories microcontroller C8051F320 with the embedded radio Texas
% Instruments CC2500 (nodes) for matlab to master node serial communication
% and master node to the car-slave nodes communication.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Clear everything
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Car id's
idr=1;
idg=2;
idye=3;
idmag=4;
% Camera Resolution
xres=1280;
yres=720;
% Distance Percentage Between Cars
distp=0.1;  % 20% distance between the cars
% The accuracy of the algorithm
accuracy=10; % cm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization Phase
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Camera init
cam = webcam('Live!');      	%Open the camera
cam.Resolution = [num2str(xres) 'x' num2str(yres)]; %Set Resolution
%Serial init
s1 = serial('COM4','BaudRate',9600,'DataBits',8);
fopen(s1); %%opening s1 file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UI (Shape Desision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while(1)
    
    %Finish Flags
    rflag=0;
    gflag=0;
    yeflag=0;
    magflag=0;
    %Number of cars
    n=input('Insert the number of cars: \n');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2 CAR SHAPES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(n==2)
        
        while(1)
            choice=input('Press : \n1 to make a vertical line \n2 to make an horizontal line \n3 to make a line in the main diagonal \n4 to make a line in the reverse diagonal\n5 to recieve car data\n0 to Exit\n');
            if choice<6 && choice>-1
                break;
            end
        end
        
        if(choice==1) % vertical line
            targetX1=xres/2;
            targetX2=xres/2;
            targetY1=yres/2 + yres*distp;
            targetY2=yres/2 - yres*distp;
        elseif(choice==2) % horizontal line
            targetX1=xres/2 + xres*distp;
            targetX2=xres/2 - xres*distp;
            targetY1=yres/2;
            targetY2=yres/2;
        elseif(choice==3) % main diag line
            targetX1=xres*distp;
            targetX2=xres - xres*distp;
            targetY1=yres*distp;
            targetY2=yres - yres*distp;
        elseif(choice==4) % secondary diag line
            targetX1=xres - xres*distp;
            targetX2=xres*distp;
            targetY1=yres*distp;
            targetY2=yres - yres*distp;
        elseif(choice==5) % data from the cars
            Car2Gateway(2,s1);
        elseif(choice==0)
            break;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 3 CAR SHAPES
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif(n==3)
        
        while(1)
            choice=input('Press : \n1 to make the triangle \n2 to recieve car data\n0 to Exit\n');
            if choice<3 && choice>-1
                break;
            end
        end
        
        if(choice==1) %Triangle
            targetX1=xres/2;
            targetX2=xres/4;
            targetX3=xres*3/4;
            targetY1=yres/4;
            targetY2=yres/2;
            targetY3=yres/2;
        elseif(choice==2) % data from the cars
            Car2Gateway(3,s1);
        elseif(choice==0)
            break;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 4 CAR SHAPES
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif(n==4)
        
        while(1)
            choice=input('Press : \n1 to make a square\n2 to make a diamond\n3 to make a line\n4 to receive car data   \n0 to Exit\n');
            if choice<5 && choice>-1
                break;
            end
        end
        
        if(choice==1)   % Square
            targetX1=xres/4+xres*distp;
            targetX2=xres/4+xres*distp;
            targetX3=xres*3/4-xres*distp;
            targetX4=xres*3/4-xres*distp;
            targetY1=yres/4;
            targetY2=yres*3/4;
            targetY3=yres/4;
            targetY4=yres*3/4;
        elseif(choice==2) % Diamond
            targetX1=xres/2-xres*distp*2;
            targetX2=xres/2+xres*distp*2;
            targetX3=xres/2;
            targetX4=xres/2;
            targetY1=yres/2;
            targetY2=yres/2;
            targetY3=yres/2+yres*distp*2;
            targetY4=yres/2-yres*distp*2;
        elseif(choice==3) % Line
            targetX1=xres/4;
            targetX2=xres/4+xres*distp;
            targetX3=xres/4+xres*distp*2;
            targetX4=xres/4+xres*distp*3;
            targetY1=yres/2;
            targetY2=yres/2;
            targetY3=yres/2;
            targetY4=yres/2;
        elseif(choice==4) % data from the cars
            Car2Gateway(4,s1);
        elseif(choice==0)
            break;
        end
        
    else
        error('This algorithm works with 2 3 and 4 cars')
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Main Algorithm
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % We take snapshots until we detect the white ruler
    while(1)
        Im=snapshot(cam);
        
        % Distance Calibration (The white 30 cm ruler can be placed both
        % verticaly and horizontaly and cars should not cover the ruler at
        % the first round)
        
        gIm=rgb2gray(Im);
        % We find the white ruler in our image
        whitemaskbf = double(gIm) > 220;
        % We connect the ruler parts if they are detected separated
        se1 = strel('square',20);
        whitemask = imclose(whitemaskbf,se1);
        % We add a morphological filter in order to get rid of the salt noise
        whitemask = bwareaopen(whitemask,1000);
        
        
        labeledImage = logical(whitemask);
        measurementsw = regionprops(labeledImage, 'Centroid');
        [sw,~]=size(measurementsw);
        
        %         figure
        %         imshow(whitemask)
        
        if(sw==1) % We have detected the white ruler
            
%             figure
%             imshow(Im)
%             
%             figure
%             subplot(2,1,1)
%             imshow(whitemaskbf);title('whitemask filtering');ylabel('Before');
%             subplot(2,1,2)
%             imshow(whitemask);ylabel('After');
            
            % We find the edges of the ruler
            [edgeY, edgeX] = find(edge(whitemask));
            % x pixels = 30 cm
            cm30inPixel=max([max(edgeX)-min(edgeX),max(edgeY)-min(edgeY)]);
            % 1 pixel = x cm
            pixel1Tocm=30/cm30inPixel;
            
            break;
            
        end
        
        % End of Distance calibration
    end
    
    while(1)
        
        tic
        
        if(n==2)
            
            [fdistancer,fdistanceg,Angler,Angleg] = Two_Car_Direction_Angle(cam,targetX1,targetX2,targetY1,targetY2,pixel1Tocm)
            toc
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % SEND DISTANCE AND ANGLE TO SERIAL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % If the car is close to the spot (spot+accuracy) we declare
            % that it has reached its destination
            if(fdistanceg<=accuracy)
                gflag=1;
            else
                gflag=0;
            end
            
            if(fdistancer<=accuracy)
                rflag=1;
            else
                rflag=0;
            end
            
            % If the car hasn't reached the spot we send the data to the car
            if(gflag==0)
                sendSerial(s1,idg,Angleg,fdistanceg)
            end
            
            if(rflag==0)
                sendSerial(s1,idr,Angler,fdistancer)
            end
            
            % The cars have reached their destination so we break the loop
            if((rflag==1)&&(gflag==1))
                disp('The cars have reached their destination !!!')
                break;
            end
            % round pause
            pause
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CASE WITH 3 CARS
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif(n==3)
            
            [fdistancer,fdistanceg,fdistanceye,Angler,Angleg,Angleye] = Three_Car_Direction_Angle(cam,targetX1,targetX2,targetX3,targetY1,targetY2,targetY3,pixel1Tocm)
            toc
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % SEND DISTANCE AND ANGLE TO SERIAL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % If the car is close to the spot (spot+accuracy) we declare
            % that it has reached its destination
            if(fdistanceg<=accuracy)
                gflag=1;
            else
                gflag=0;
            end
            
            if(fdistancer<=accuracy)
                rflag=1;
            else
                rflag=0;
            end
            
            if(fdistanceye<=accuracy)
                yeflag=1;
            else
                yeflag=0;
            end
            
            % If the car hasn't reached the spot we send the data to the car
            
            if(gflag==0)
                sendSerial(s1,idg,Angleg,fdistanceg)
            end
            
            if(rflag==0)
                sendSerial(s1,idr,Angler,fdistancer)
            end
            
            if(yeflag==0)
                sendSerial(s1,idye,Angleye,fdistanceye)
            end
            
            % The cars have reached their destination so we break the loop
            
            if((rflag==1)&&(gflag==1)&&(yeflag==1))
                disp('The cars have reached their destination !!!')
                break;
            end
            % round pause
            pause
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CASE WITH 4 CARS
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        else
            [fdistancer,fdistanceg,fdistanceye,fdistancemag,Angler,Angleg,Angleye,Anglemag] = Four_Car_Direction_Angle(cam,targetX1,targetX2,targetX3,targetX4,targetY1,targetY2,targetY3,targetY4,pixel1Tocm,whitemask)
            toc
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % SEND DISTANCE AND ANGLE TO SERIAL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % If the car is close to the spot (spot+accuracy) we declare
            % that it has reached its destination
            if(fdistanceg<=accuracy)
                gflag=1;
            else
                gflag=0;
            end
            
            if(fdistancer<=accuracy)
                rflag=1;
            else
                rflag=0;
            end
            
            if(fdistanceye<=accuracy)
                yeflag=1;
            else
                yeflag=0;
            end
            
            if(fdistancemag<=accuracy)
                magflag=1;
            else
                magflag=0;
            end
            
            % If the car hasn't reached the spot we send the data to the car
            
            if(gflag==0)
                sendSerial(s1,idg,Angleg,fdistanceg)
            end
            
            if(rflag==0)
                sendSerial(s1,idr,Angler,fdistancer)
                
            end
            
            if(yeflag==0)
                sendSerial(s1,idye,Angleye,fdistanceye)
                
            end
            
            if(magflag==0)
                sendSerial(s1,idmag,Anglemag,fdistancemag)
            end
            
            % The cars have reached their destination so we break the loop
            
            if((rflag==1)&&(gflag==1)&&(yeflag==1)&&(magflag==1))
                disp('The cars have reached their destination !!!')
                break;
            end
            % round pause
            pause
            
        end
        
    end
    
end
% Delete the serial connection after we finish
delete(instrfindall)