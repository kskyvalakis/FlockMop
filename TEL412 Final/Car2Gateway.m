function [] = Car2Gateway(num,s1)

%always use flushinput to clear the stream
%between matlab and master-radio
flushinput(s1)

%sending 
for i=1:num
    if i==1
        sendSerial(s1,i+4,50,150)
        flushinput(s1)
    elseif i==2
        sendSerial(s1,i+4,60,160)
        flushinput(s1)
    elseif i==3
        sendSerial(s1,i+4,70,170)
        flushinput(s1)
    else
        sendSerial(s1,i+4,80,180)
        flushinput(s1)
    end
end

end