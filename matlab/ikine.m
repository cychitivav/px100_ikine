function q = ikine(T)
    L1 = 0.0445; %m
    L2 = 0.1010; %m
    L3 = 0.1010; %m
    L4 = 0.1090; %m (TCP)
    Lm = 0.0315; %m  

    q = zeros(1,4);

    % q1 (Waist)
    q(1) = atan2(T(2,4),T(1,4));

    % 2R mechanism
    a = T(1:3,3);
    w = T(1:3,4)-L4*a;

    r = sqrt(w(1)^2+w(2)^2);
    h = w(3)-L1;

    hyp = sqrt(r^2+h^2);

    beta = atan2(Lm,L2);
    psi = pi/2-beta;
    Lr = sqrt(Lm^2+L2^2);

    phi = acos((hyp^2-L3^2-Lr^2)/(-2*Lr*L3));

    gamma = atan2(h,r);
    alpha = acos((L3^2-Lr^2-hyp^2)/(-2*Lr*hyp));

    % q3 (Elbow)
    q(3) = pi - psi-phi;

    % q2 (Shoulder)
    q(2) = pi/2-beta-alpha-gamma;

    % q4 (Wrist)
    eul = tr2eul(T);

    q(4) = eul(2)-q(2)-pi/2-q(3);
    
end