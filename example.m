% constants

T = 0.1;

% Create idealized system to match filter assumptions
% state is (position, velocity)'
% We are assuming Q, R, and P0 to be diagonal

A = [ 1 T
      0 1 ]

B = [ 0
      T ]

C = [ 1 0 ]

% process variance
Q = [ 1e-6 0
      0 1e-5 ]

% sensor noise variance
R = [ 1e-5 ]

% initial state estimate variance
P0 = [ 1e-4 0
       0 1e-4 ]

% Create some data
measurement = [0.00 -71.28 -149.07 -226.85 -294.51 -355.30 -425.49 -497.49 -571.30 -641.13 -693.59 -761.97 -838.31 -913.21 -989.19 -1050.33 -1115.82 -1186.01 -1258.37 -1329.65 -1392.60 -1451.94 -1517.43 -1583.64 -1655.28 -1726.19 -1790.59 -1844.50 -1908.90 ]

%create speed from taking difference
diff = []
for i = 1:(length(measurement) - 1)
    diff(i) = (measurement(i + 1) - measurement(i)) / T
end
% Design filter
%v Note that we can design filter in advance of seeing the data.
Pm = P0;
for i = 1:1000
 % measurement step
 S = C*Pm*C' + R;
 K = Pm*C'*inv(S);
 Pp = Pm - K*C*Pm;
 % prediction step
 Pm = A*Pp*A' + Q;
end

% Run the filter to create example output
sem = [ 0 0 ]'
for i = 1:length(measurement - 1)
 % measurement step
 sep = sem + K*(measurement(i)-C*sem);
 pose(i) = sep(1);
 vele(i) = sep(2);	   
 % prediction step
 sem = A*sep + B*-50;
end

% Let's plot the Kalman filter output
ii = 1:length(measurement - 1);
%plot(ii,pose,'b')
%plot(ii,measurement,'b')
%plot(ii,vele,'b')
%legend('KF velocity','true velocity')
%xlabel('sample (100Hz)')
plot(diff, 'r')
plot(vele, 'b')
x =1:1:length(diff);
plot(x,vele(1:length(diff)),'r',x,diff,'b', x, measurement(1:length(diff)), 'g');
