%% TODO:
% Продолжение по параметру
%
%%
velocity =  1.388*randB(0.6, 2);
wheelBase = 2.6*randB(0.5, 2); 
maxSteeringAngle = deg2rad(20);
maxSteeringVelocity = 0.34;

initXPos = 0;
initYPos = randB(-10, 10);

initHeading = deg2rad(randB(-180, 180));
initSteeringAngle = maxSteeringAngle*0.95*rand;
targetYPos = 0; 

R = wheelBase/maxSteeringAngle;
T_ch = 2*R/velocity;

global gFI

global gStates;
gStates = [];

clc
% hold on
% CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
%                 targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);
             

% CPF = clothoidPathFinder(0, 4.6344477131734063135581891401671, 0.92831526477313930367785133057623, 0.14953196369781002195153973843844,...
%                 0, 3.831550352330163278224972600583, 1.1611814252519270240782134351321, 0.34906585039886591538473815369772, 0.34);

 CPF = clothoidPathFinder(0,  -0.88437557220458984375,  0.44361143965993349302934234401619,  0.031834669944260728102758406521389,...
 0, 1.3445627726614475250244140625,  1.4839250772953032075918144983007,  0.34906585039886589560964580414293 , 0.344);


global RESULTS
CPF.isDrawDubins = 0;
CPF.isDrawFirstNumerical = 0;
CPF.isDrawArrows = 0;   
 
[ret_x, ret_y, ret_th, ret_fi] = CPF.findPath();

RESULTS = [RESULTS; [norm([ret_y, sin(0.5*ret_th), ret_fi]), [ret_y, sin(0.5*ret_th), ret_fi], [initXPos, initYPos, initHeading, initSteeringAngle], [targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity]]];
disp(norm([ret_y, sin(ret_th), ret_fi]));
disp(vpa([initXPos, initYPos, initHeading, initSteeringAngle]));
disp(vpa([wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity]));

function num = randB(a, b)
    num = a + (b - a) * rand();
end
