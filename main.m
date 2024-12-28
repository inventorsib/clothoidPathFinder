%% TODO:
% Продолжение по параметру
%
%%
initXPos = 0;
initYPos = 22.8468;
% global gFI
initHeading = 2.6131; 
initSteeringAngle = -0.0074; 
targetYPos = 0; 

velocity =  1.388;
wheelBase = 2.6; 
maxSteeringAngle = deg2rad(20);
maxSteeringVelocity = 1+0.34;

clc
CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
                targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);

CPF.pathTimeStepDivider = 5000;
CPF.isDrawDubins = 1;
CPF.isDrawFirstNumerical = 1;
CPF.isDrawArrows = 1;   

% [resultsOfTest, initStates, T, C] = CPF.test(2000);
% plot(resultsOfTest, 'LineWidth', 2); grid on; grid minor;
 
CPF.findPath();

