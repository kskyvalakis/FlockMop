function [] = sendSerial(s1,id,deg,cm)
%A function that sends the data from the pc serial port to the master node
pause(1)

flag=0;

if id>4 && id<9
    flag=1;
end
%this is needed in order to use one extra
%bit for deg and one extra for cm in the id
%(we have 8bits each but id only uses 3 of his
%own ,so we give the msb to deg and the one 
%right to it to cm)
if deg>255 
    id=id+128;
    deg=deg-256;
end 

if cm>255
    id=id+64;
    cm=cm-256;
end
    


pause(0.01)
fwrite(s1,id);
pause(0.1)
fwrite(s1,deg);
pause(0.1)
fwrite(s1,cm);



if flag==1
  
    pause(2)   
  
  fprintf('Car %d measurements:\n',id-4)
  fscanf(s1,'%d')
  fscanf(s1,'%d',8)
  fscanf(s1,'%d',8)
  
end


