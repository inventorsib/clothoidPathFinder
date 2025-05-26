%% TODO:
% Продолжение по параметру
%
%%
velocity =  1.388*randB(0.5, 2);
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
CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
                targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);

global RESULTS
CPF.isDrawDubins = 0;
CPF.isDrawFirstNumerical = 0;
CPF.isDrawArrows = 0;   

% [resultsOfTest, initStates, T, C] = CPF.test(2000);
% plot(resultsOfTest, 'LineWidth', 2); grid on; grid minor;
 
[ret_x, ret_y, ret_th, ret_fi] = CPF.findPath();

RESULTS = [RESULTS; [norm([ret_y, sin(0.5*ret_th), ret_fi]), [ret_y, sin(0.5*ret_th), ret_fi], [initXPos, initYPos, initHeading, initSteeringAngle], [targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity]]];
disp(norm([ret_y, ret_th, ret_fi]));

function num = randB(a, b)
    num = a + (b - a) * rand();
end
