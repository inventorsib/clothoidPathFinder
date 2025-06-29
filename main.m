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


global gStates;
gStates = [];

clc
% hold on

% CPF = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
%                 targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity);
CPF = clothoidPathFinder(0, 4.4719, -0.73516, 0.098826, 0, 4.828, 2.4512, 0.34907, 0.34);
% CPF = clothoidPathFinder(0, 25, 0.0, 0.0, ...
%                 0, 3.831550352330163278224972600583, 1.1611814252519270240782134351321, 0.34906585039886591538473815369772, 0.34);

% CPF = clothoidPathFinder(0, 4.6344477131734063135581891401671, 0.92831526477313930367785133057623, 0.14953196369781002195153973843844,...
%                 0, 3.831550352330163278224972600583, 1.1611814252519270240782134351321, 0.34906585039886591538473815369772, 0.34);

%  CPF = clothoidPathFinder(0,  -0.88437557220458984375,  0.44361143965993349302934234401619,  0.031834669944260728102758406521389,...
%  0, 1.3445627726614475250244140625,  1.4839250772953032075918144983007,  0.34906585039886589560964580414293 , 0.344);


% clf
tsum_p_0 = CPF.findPath(1, 0, 0);
tsum_m_0 = CPF.findPath(-1, 0, 0);
tsum_p_p2pi = CPF.findPath(1, 0, 2*pi);
tsum_m_p2pi = CPF.findPath(-1, 0, 2*pi);
tsum_p_m2pi = CPF.findPath(1, 0, -2*pi);
tsum_m_m2pi = CPF.findPath(-1, 0, -2*pi);

quality = [tsum_p_0, tsum_m_0, tsum_p_p2pi, tsum_m_p2pi, tsum_p_m2pi, tsum_m_m2pi];

[val, indx] = min(quality);

% hold on
% CPF.findPath(1, 1, 0);
% CPF.findPath(-1, 1, 0);
% CPF.findPath(1, 1, 2*pi);
% CPF.findPath(-1, 1, 2*pi);
% CPF.findPath(1, 1, -2*pi);
% CPF.findPath(-1, 1, -2*pi);
% hold off

switch indx
    case 1
        CPF.findPath(1, 1, 0);
    case 2
        CPF.findPath(-1, 1, 0);
    case 3
        CPF.findPath(1, 1, 2*pi);
    case 4
        CPF.findPath(-1, 1, 2*pi);
    case 5
        CPF.findPath(1, 1, -2*pi);
    case 6
        CPF.findPath(-1, 1, -2*pi);
end

% global g_counter
% exportgraphics(gcf, num2str(g_counter)+".jpg", 'Resolution', 150, 'BackgroundColor','white');
% 
% vector = [initXPos, initYPos, initHeading, initSteeringAngle,...
%                 targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity];
%             
% dlmwrite('log.txt', [g_counter vector], '-append', 'delimiter', ' ', 'newline', 'pc');
% g_counter = g_counter + 1;

function num = randB(a, b)
    num = a + (b - a) * rand();
end
