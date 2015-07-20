TargetEncoder = 10000;

% manual tuning
Kp = 0.3;
Ki = 0;
Kd = 0.8; 

MotorVelocity = 500;
ExtraTime = 100;

[tt, LEncoder, REncoder, TargetEncoderVec, LVel, RVel, LError, RError] = DE2Bot(TargetEncoder, Kp, Ki, Kd, MotorVelocity, ExtraTime);

figure(1);
clf(1);
hold on;
xlabel('Motor control ticks (0.1s)');
ylabel('Motor encoder value');
plot(tt, LEncoder, 'b');
plot(tt, REncoder, 'r');
plot(tt, TargetEncoderVec, 'g');

%plot(tt, LError, 'c');
%plot(tt, RError, 'm');

legend('Left Encoder', 'Right Encoder', 'Target Encoder', 'Left Error', 'Right Error');