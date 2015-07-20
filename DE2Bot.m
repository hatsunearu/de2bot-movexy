function [tt, LEncoder, REncoder, TargetEncoderVec, LVel, RVel, LError, RError] = DE2Bot(TargetEncoder, Kp, Ki, Kd, MotorVelocity, ExtraTime) 

    Amax = 51.2; % maximum acceleration/deceleration
    %(512 units/s) * (1/10 second/tick)
    
    tt = 0:TargetEncoder*ExtraTime/MotorVelocity; % make tick vector, 10Hz sampling
    
    numSamples = length(tt);
    
    TargetEncoderVec = ones(1,numSamples) * TargetEncoder; % target encoder value
    TargetEncoderVec(1:floor(numSamples*0.1)) = 0; % start stationary
    
    LEncoder = zeros(1,numSamples);
    REncoder = zeros(1,numSamples);
    LVel = zeros(1, numSamples);
    RVel = zeros(1, numSamples);
    
    LError = zeros(1,numSamples);
    RError = zeros(1,numSamples);
    LErrorIntegral = zeros(1, numSamples+1);
    RErrorIntegral = zeros(1, numSamples+1);
    LErrorDeriv = zeros(1, numSamples+1);
    RErrorDeriv = zeros(1, numSamples+1);
    
     Lowpasskernel = 0.1*ones(1,10);
     LMotorError = conv((1 - 0.05*rand(1,numSamples)), Lowpasskernel);
     RMotorError = conv((1 - 0.05*rand(1,numSamples)), Lowpasskernel);
     
    for i = 2:numSamples-1
     
        %update K error
        LError(i) = TargetEncoderVec(i) - LEncoder(i);
        RError(i) = TargetEncoderVec(i) - REncoder(i);
        
        %update I error
        LErrorIntegral(i+1) = LErrorIntegral(i) + LError(i);
        RErrorIntegral(i+1) = RErrorIntegral(i) + RError(i);
        
        %update D error
        LErrorDeriv(i) = LError(i) - LError(i-1);
        RErrorDeriv(i) = RError(i) - RError(i-1);
        
        %Apply PID control
        LVel(i+1) = LError(i) * Kp + LErrorIntegral(i) * Ki  + LErrorDeriv(i) * Kd;
        RVel(i+1) = RError(i) * Kp + RErrorIntegral(i) * Ki  + RErrorDeriv(i) * Kd;
        
        %Acceleration limiting
        if LVel(i+1) - LVel(i) > Amax || LVel(i+1) - LVel(i) < -Amax
            LVel(i+1) = LVel(i) + sign(LVel(i+1) - LVel(i)) * Amax;
        end
        if RVel(i+1) - RVel(i) > Amax || RVel(i+1) - RVel(i) < -Amax
            RVel(i+1) = RVel(i) + sign(RVel(i+1) - RVel(i)) * Amax;
        end
        
        %Motor Velocity limiting
        if abs(LVel(i+1)) > MotorVelocity
            LVel(i+1) = sign(LVel(i+1)) * MotorVelocity;
        end
        if abs(RVel(i+1)) > MotorVelocity
            RVel(i+1) = sign(RVel(i+1)) * MotorVelocity;
        end
        
        
        %apply encoder displacement w/ motor errors
        LEncoder(i+1) = LVel(i) * LMotorError(i) + LEncoder(i);
        REncoder(i+1) = RVel(i) * RMotorError(i) + REncoder(i);
        
    end
    
end