%setup the Arduino object and establish the USB connection
%the arguement to Arduino needs to be the name of the usb serial connection
a = Arduino("/dev/cu.usbmodem1411");
a.sendXY(0,0);
axes('pos', [0 0 1 1]);
axis([0 240 0 240]);

%determines the size of each sqaure in the 4 X 4 grid
gridSize = 80;

for i = (0:3)
    for j = (0:3)
        values = a.sendXY(gridSize * i, gridSize * j);
        
        %draw all of the vectors
        for k = (1:8)
            
        end
    end         
end
end

delete(a);

