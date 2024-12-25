%% TODO:

%%
initXPos = 0;
initYPos = 0;
initHeading = 3.14; 
initSteeringAngle = 0; 
targetYPos = 5; 

velocity =  1.3889;
wheelBase = 2.6; 
maxSteeringAngle = 0.5;
maxSteeringVelocity = 10.34;

clc
CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
                targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);

CPF.pathTimeStepDivider = 15000;
CPF.isDrawDubins = 1;
CPF.isDrawFirstNumerical = 1;
CPF.isDrawArrows = 1;   

% [resultsOfTest, initStates, T, C] = CPF.test(2000);
% plot(resultsOfTest, 'LineWidth', 2); grid on; grid minor;
 
[x, y] = CPF.findPath();

