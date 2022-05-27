function q = ikine(T, varargin)
% IKINE Returns the position of the joints to reach the desired position 
% T of the TCP
%
%   q = ikine(T) are the joint coordinates (1x4) corresponding to the  
%   pincher x100 robot end-effector pose T (4x4) which is a homogenenous 
%   transform.
%
%   q = ikine(...,OPTION,Value) 
% 
%   OPTIONS:
%   'lenghts', L 
%   'elbow', E

    L1 = 0.0445; %m
    L2 = 0.1010; %m
    L3 = 0.1010; %m
    L4 = 0.1090; %m (TCP)
    Lm = 0.0315; %m 

    up = true;

    if nargin > 1
        if nargin ~= 3 && nargin ~= 5
            error('Bad number of input arguments.')
        end

        if any(strcmp(varargin,'elbow'))
            idx = find(strcmp(varargin,'elbow'));

            if strcmp(varargin{idx+1},'up')
                up=true;
            elseif strcmp(varargin{idx+1},'down')
                up=false;
            else
                error("Bad configuration. Possible options 'up' or 'down'")
            end
        elseif any(strcmp(varargin,'lengths'))
            idx = find(strcmp(varargin,'lengths'));
            L = varargin{idx+1};

            L1 = L(1);
            L2 = L(2);
            L3 = L(3); 
            L4 = L(4); 
            Lm = L(5); 
        else
            error(varargin{1} + 'unknowm.')
        end
    end

    q = zeros(1,4);

    % q1 (Waist)
    q(1) = atan2(T(2,4),T(1,4));

    % Wrist decoupling
    a = T(1:3,3);
    w = T(1:3,4)-L4*a;

    % 2R mechanism
    r = sqrt(w(1)^2+w(2)^2);
    h = w(3)-L1;

    c = sqrt(r^2+h^2);

    beta = atan2(Lm,L2);
    psi = pi/2-beta;
    Lr = sqrt(Lm^2+L2^2);

    phi = acos((c^2-L3^2-Lr^2)/(-2*Lr*L3));

    gamma = atan2(h,r);
    alpha = acos((L3^2-Lr^2-c^2)/(-2*Lr*c));

    % q2 (Shoulder)
    if up
        q(2) = pi/2-beta-alpha-gamma;
    else
        q(2) = pi/2-(gamma-alpha+beta);
    end

    % q3 (Elbow)
    if up
        q(3) = pi - psi-phi;
    else
        q(3) = -pi+(phi-psi);
    end

    % q4 (Wrist)
    eul = tr2eul(T);

    q(4) = eul(2)-q(2)-pi/2-q(3);
    
    q = real(q);
end