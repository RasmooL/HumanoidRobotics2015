function pos = rightarmnormalized(s, q1, q2, q3, q4, q5)
    rightarm = evalin('base', 'rightarm');
    UpperArmLength = evalin('base', 'UpperArmLength');
    LowerArmLength = evalin('base', 'LowerArmLength');
    HandOffsetX = evalin('base', 'HandOffsetX');
    
    L = UpperArmLength + LowerArmLength + HandOffsetX;
    NUpperArmLength = UpperArmLength / L;
    NLowerArmLength = LowerArmLength / L;
    NHandOffsetX = HandOffsetX / L;
    
    if s < NUpperArmLength
        s = s / NUpperArmLength; % normalize parameter s
        A = rightarm.A([1 2 3], [q1 q2 q3]);
        elbow_pos = A(1:3, 4);
        pos = elbow_pos/L * s;
    else%if s < NUpperArmLength + NLowerArmLength % combine wrist for now (dont care about wrist rotation)
        s = (s - NUpperArmLength) / (NLowerArmLength + NHandOffsetX);
        A = rightarm.A([1 2 3], [q1 q2 q3]);
        elbow_pos = A(1:3, 4);
        A = rightarm.A([1 2 3 4 5], [q1 q2 q3 q4 q5]);
        wrist_pos = (A(1:3, 4) - elbow_pos)/L + elbow_pos/L;
        elbow_pos = elbow_pos/L;
        pos = wrist_pos * s + (1-s)*elbow_pos;
    %else
        
    end
    pos = pos;

end

