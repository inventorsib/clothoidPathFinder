%% TODO:
% Продолжение по параметру
%
%%
initXPos = 0;
initYPos =  rand*3*3.3536;%4.3086;
global gFI
initHeading =  2.1354*0 + pi*rand; %1.2217;
initSteeringAngle =  0.0119*rand; %0.0057;
targetYPos = 0; 

velocity =  1.388;
wheelBase = 2.6; 
maxSteeringAngle = deg2rad(20);
maxSteeringVelocity = 0.34;

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

