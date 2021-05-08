function w = BangBangTurnRateController(thCmd, thCurr, thDotMax)
%BANGBANGTURNRATECONTROLLER Summary of this function goes here
%   Detailed explanation goes here
    
    % Calculate error and sign
    error1 = thCmd - thCurr;
    error2 = 2*pi - abs(error1);
    if abs(error1) <= abs(error2)
        sign_a = sign(error1);
        error_true = error1;
    else
        sign_a = -sign(error1);
        error_true = -sign(error1)*error2;
    end
    if abs(error_true) < 1e-6
        sign_a = 0;
    end
    
    w = thDotMax * sign_a;
end

