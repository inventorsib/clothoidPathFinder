global l v max_u max_fi
l = 1;
v = 1;

max_u = 1;
max_fi = 0.15;

h = 0.05;
t = 0;

R = [0, -0.5; 0, 0.5; 2, 0.45; 2, -0.45];
Rw = [-1, -0.5; -1, 0.5; 1, 0.5; 1, -0.5]*0.3;
Rw1 = [2, -0.5; 2, 0.5; 4, 0.5; 4, -0.5]*0.2;
 
 state = [0 5 0 0];

while t<13
%     [~,~,b] = ginput(1)
   clf
   
   [x, y, th, fi] = state2coordinates(state);
   fi = constrain(fi, max_fi, -max_fi);
   
   CPF = clothoidPathFinder(x, y, th, fi,...
                0, 1, 1, max_fi, 1);

   
   hold on
   
   xlim([-15 15]);
   ylim([-15 15]);
   
   [X, Y, c] = CPF.findPath();
   control = c;
   
   [dState] = bicycleODE(state, control);
   state = state + dState*h;
   [x, y, th, fi] = state2coordinates(state);
   fi = constrain(fi, max_fi, -max_fi);
   
   romat = @(a)[cos(a), -sin(a); sin(a), cos(a)];
   Rrot = (R*romat(-th));
   
%     plot([-10000 10000], [0 0]);
%     plot(X, Y);


   axis equal
   patch(Rrot(:,1)+x,Rrot(:,2)+y,'green')
   quiver(x, y, 5*cos(th+fi), 5*sin(th+fi), 'LineWidth', 2.1);
   
   
   grid on
   grid minor
   
   t = t + h;
   drawnow;
end



%%BICYCLEODE
function [dState] = bicycleODE(state, control)
    %BICYCLEODE 
        global l v max_u max_fi
        u = control(1);

        [x, y, th, fi] = state2coordinates(state);

        u  = constrain(u, max_u, -max_u); 
        fi = constrain(fi, max_fi, -max_fi);

        dState(1, 1) = v * cos(th);
        dState(2, 1) = v * sin(th);
        dState(3, 1) = v/l * tan(fi); % TODO: to be precise tan(fi)
        dState(4, 1) = u;

end
    
function [x, y, th, fi] = state2coordinates(state)

    x = state(1);
    y = state(2);
    th = state(3);
    fi = state(4);

end

function [outVal] = constrain(inVal, maxVal, minVal)
%CONSTRAIN 
%   input value
%   maximum available value
%   minimum available value

    if inVal>maxVal
        outVal = maxVal;
        return;
    end
    if inVal<minVal
        outVal = minVal;
    else
        outVal = inVal;
    end

end