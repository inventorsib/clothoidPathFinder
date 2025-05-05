%% TODO:
% Продолжение по параметру
%
%%
initXPos = 0;
initYPos =  0.5;
global gFI
initHeading = -0.2268928;
initSteeringAngle =  -0.29999;
targetYPos = 0; 

velocity =  2.6372; %1.388;
wheelBase = 2.6; 
maxSteeringAngle = deg2rad(20);
maxSteeringVelocity = 0.34*0.5;

global gStates;
gStates = [];

clc
CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
                targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);

CPF.pathTimeStepDivider = 5000;
CPF.isDrawDubins = 0;
CPF.isDrawFirstNumerical = 0;
CPF.isDrawArrows = 0;   

% [resultsOfTest, initStates, T, C] = CPF.test(2000);
% plot(resultsOfTest, 'LineWidth', 2); grid on; grid minor;
 
CPF.findPath();

