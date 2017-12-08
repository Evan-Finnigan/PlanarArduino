function []  = statuscallback( obj, event, arduino)
%STATUSCALLBACK Summary of this function goes here
%   Detailed explanation goes here
    recievedStr = strip(fscanf(arduino.serialConnection));
    if(recievedStr(1) == 'd')
        arduino.done = true;
        arduino.endPositions = strsplit(recievedStr(2:end), ',');
    else
        %take off uneeded whitespace
        strippedStr = strip(recievedStr);
        arduino.setData(strsplit(strippedStr, ","));
    end
end

