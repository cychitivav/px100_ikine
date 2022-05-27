clc, clear, close all
%% Robot parameters
q1 = 1;
q2 = 1;
q3 = 1;
q4 = 1;

L1 = 0.0445; %m
L2 = 0.1010; %m
L3 = 0.1010; %m
L4 = 0.1090; %m (TCP)
Lm = 0.0315; %m  
%% Robot creation 
% DH = [THETA D A ALPHA SIGMA OFFSET] 
DH = [q1	L1	0	            -pi/2	0	0;
      q2	0	sqrt(L2^2+Lm^2)	0	    0	-atan(L2/Lm);
      q3	0	L3	            0	        0	atan(L2/Lm);
      q4	0	L4	            0	    0	0];

for i=1:size(DH)
    L(i) = Link(DH(i,:));
end

PX = SerialLink(L,'name','Filoberta','tool',trotx(-pi/2)*troty(pi/2))
%% Fkine
qf = [0 0 -pi/2 0]
TCP = PX.fkine(qf);
%% Ikine
qup = ikine(TCP,'elbow','up')
qdo = ikine(TCP,'elbow','down')
%% Plots
figure
trplot(PX.fkine(qf),'rgb','length',0.1,'thick',2)
view(30,30)
axis tight
hold on
PX.plot([qf;qup;qdo],'noa','jaxes','notiles','floorlevel',0,'noshadow','delay',1)
axis tight
