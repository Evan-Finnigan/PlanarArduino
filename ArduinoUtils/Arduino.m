classdef Arduino < handle
    %ARDUINO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        comPort
        serialConnection
        done
        data
        endPositions
    end
    
    methods
        %constructor will set the compPort and open the connection
        function obj = Arduino(inputComPort)
            obj.comPort = inputComPort;
            % create the serial connection to the arduino
            obj.serialConnection = serial(obj.comPort);
            obj.serialConnection.BytesAvailableFcnMode = 'terminator';
            obj.serialConnection.BytesAvailableFcn = {@statuscallback, obj};
            set(obj.serialConnection,'BaudRate',115200);
            fopen(obj.serialConnection);
        end
        
        %function to send xy coordinates
        function endPositions = sendXY(obj, x, y)
            sendStr = strcat("s", num2str(x), ",", num2str(y));
            fprintf(obj.serialConnection, sendStr);
            
            obj.done = false;
            %wait until the stage has reached the last setpoint
            while ~obj.done
                pause(.001);
            end
            
            endPositions = obj.endPositions;
        end
       
        %function to read x, y coordinates
        function readXY(obj)
           fprintf(obj.serialConnection, "r\n");
        end
        
        function dispData(obj)
            disp(obj.data)
        end
        
        function setData(obj, newData)
            obj.data = newData;
        end
       
        %destructor will close the com port
        function delete(obj)
            disp("Deleting arduino");
            fclose(obj.serialConnection);
            delete(obj.serialConnection);
            clear obj.serialConnection;
        end
    end
    
end

