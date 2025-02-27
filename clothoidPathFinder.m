classdef clothoidPathFinder
    %CLOTHOIDPATHFINDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % transport parameters
        velocity
        wheelBase
        maxSteeringAngle
        maxSteeringVelocity
        
        % initial conditions
        initSteeringAngle
        initXPos
        initYPos
        initHeading
        
        % target
        targetYPos
        xBestDubins
        
        % results containers
        t_arr
        pos_x
        pos_y
        fi_arr
        th_arr

        u_arr

        x_dir_arr
        y_dir_arr
        
        % other pathfinder parameters
        pathTimeStepDivider
        controlVector
        
        % private variables
        p_parameter
        p_dfi
        p_dth
        p_dy
        p_MinTurningRadius
        
        % draw parameters
        isDrawDubins
        isDrawFirstNumerical
        isDrawArrows
        
    end
    
    methods
        function obj = clothoidPathFinder(initXPos, initYPos, initHeading, initSteeringAngle,...
                targetYPos, wheelBase, velocity, maxSteeringAngle, maxSteeringVelocity)
            %CLOTHOIDPATHFINDER Construct an instance of this class
            %
            obj.maxSteeringAngle = maxSteeringAngle;
            obj.maxSteeringVelocity = maxSteeringVelocity;
            obj.wheelBase = wheelBase;
            obj.velocity = velocity;
            
            obj.initSteeringAngle = initSteeringAngle;
            obj.initXPos = initXPos;
            obj.initYPos = initYPos;
            obj.initHeading = initHeading;
            
            obj.targetYPos = targetYPos;
            
            % flush containers
            obj.t_arr = [];
            obj.pos_x = [];
            obj.pos_y = [];
            obj.fi_arr = [];
            obj.th_arr = [];

            obj.u_arr = [];

            obj.x_dir_arr = [];
            obj.y_dir_arr = [];
            
            % set to default parameters
            obj.pathTimeStepDivider = 1000;
            
        end

        %% Find path
        function [x, y, c] = findPath(obj)
        
            % TODO: убрать холд он и глобальные переменные
            % вернуть х, у - путь
            hold on
            
            x1 = obj.getBestDubins2Line(obj.initXPos);
            obj.xBestDubins = x1;
            [times, control] = obj.getApproximationFromDubins(x1);
            [times, control] = obj.initFromTable(times, control);
            
            control = obj.maxSteeringVelocity*control;
            obj.controlVector = control;
            
%             control(4) = -control(4);
%             control(5) = -control(5);

            if obj.isDrawFirstNumerical == 1
                
                % new idea 
                times = [times(1) times(3) times(4)];
                st = times;
                t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
                times = [st(1) t0 st(2) st(3) t0];
                
                obj.buildPathNumeracly(times, control,...
                    obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
            end
            
            %% Optimization part

            options = optimoptions('fminunc', 'FiniteDifferenceStepSize', 0.01, ...
                'FiniteDifferenceType', 'central', ...
                'UseParallel', true);
            
            global DY DTH DFI
            DY = []; DTH = []; DFI = [];


            %%
            
            % new idea 
            times = [times(1) times(3) times(4)];
            tic
            for p = 10:1000:5000
                obj.p_parameter = p;
%               times = fminunc(@obj.func, times);
                times = fminsearch(@obj.func, times);
%               times = fmincon(@obj.func, times, [], [], [], [], [1 1 1]*10, [1 1 1]*20);
            end
            toc
            
            % new idea 
            st = times;
            t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
            times = [st(1) t0 st(2) st(3) t0];
            
            [x_n, y_n, th_n, fi_n] = obj.buildPathNumeracly(times, control,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalytically(times, control,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
            
            if abs(y_a)>0.5
                stp = 1;
                disp('stop')
            end
            
            disp([x_a, y_a, th_a, fi_a]-[x_n, y_n, th_n, fi_n])
            
            x = obj.pos_x; y = obj.pos_y;
            
            c = control(1);
            
        end

        %% OLD
        function [J] = func(obj, switch_times)
            
            % new idea 
            st = switch_times;
            t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
            switch_times = [st(1) t0 st(2) st(3) t0];
            
            p = obj.p_parameter;

            x0 = obj.initXPos;
            y0 = obj.initYPos;
            th0 = obj.initHeading;
            fi0 = obj.initSteeringAngle;
            
           
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalytically(switch_times, obj.controlVector, ...
                x0, y0, th0, fi0);
            
            y_target = obj.targetYPos;

            swtc = abs(min(0, switch_times));

            % TODO: задать и обосновать коэффициент
            J = p*(1*(y_a-y_target)^2 + 10*abs(sin(th_a))^2 + 0*abs(rad2deg(fi_a))^2 ...
                + 10*norm(swtc)*norm(swtc))...
                + sum(abs(switch_times));

            
            global dy dth dfi
            dy = abs(y_a-y_target);
            dth = abs(rad2deg(th_a));
            dfi = abs(rad2deg(fi_a));

            global DY DTH DFI
            DY = [DY dy]; DTH = [DTH dth]; DFI = [DFI dfi];

        end
        

        %% NEW
        function [c,ceq] = mycon(obj, switch_times)
            
            c = [];
            
            x0 = obj.initXPos;
            y0 = obj.initYPos;
            th0 = obj.initHeading;
            fi0 = obj.initSteeringAngle;
            
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalytically(switch_times, obj.controlVector, ...
                x0, y0, th0, fi0);
            
            ceq(1) = y_a;
            ceq(2) = th_a;
            ceq(3) = fi_a;

        end
        
        function [J] = func_con(obj, switch_times)
            
            J = sum(switch_times);
            
            x0 = obj.initXPos;
            y0 = obj.initYPos;
            th0 = obj.initHeading;
            fi0 = obj.initSteeringAngle;
            
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(switch_times, obj.controlVector, ...
                x0, y0, th0, fi0);
            
            J = J + y_a^2 + th_a^2 + fi_a^2;
            
            return;
        end

       
        %% Get best Dubins path to line
        function [best_x1] = getBestDubins2Line(obj, x0)
    
            obj.p_MinTurningRadius = obj.wheelBase/obj.maxSteeringAngle;

            best_x1 = fminbnd(@obj.getDubinsPathLength, x0-1*obj.p_MinTurningRadius, x0+4*obj.p_MinTurningRadius);

        end

        function [length] = getDubinsPathLength(obj, x1)
            
            th1 = 0;
            
            dubConnObj = dubinsConnection;
            
            dubConnObj.MinTurningRadius = obj.p_MinTurningRadius;
            
            startPose = [obj.initXPos obj.initYPos obj.initHeading];
            goalPose = [x1 obj.targetYPos th1];
            
            [pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);

            length = pathCosts;
            
        end
        
        %% Build path analytically
        function [x, y, th, fi] = buildPathAnalytically(obj, duration_intervals, control_vector, x, y, th, fi)

            % BUG: double multiply on maxSteeringVelocity
            % control_vector = control_vector * obj.maxSteeringVelocity;
            
            i = 1;
            
            for c = control_vector
                current_switch_time = duration_intervals(i);
                turn_sign = c;

                if c~=0
                    [x, y, th, fi, time_interval] = ...
                    obj.getKlothoideTransition(x, y, th, fi, turn_sign, current_switch_time);
                    
%                     disp(["Klothoide",  x, y, th, fi]);
                    
                    if current_switch_time>time_interval
                        residual_time_interval = current_switch_time-time_interval;
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, residual_time_interval);
                    
%                         disp(["Circle",  x, y, th, fi]);
                    end
                else
                    if abs(fi)<deg2rad(0.05)
                        [x, y, th, fi, not_used] = ...
                        obj.getLineTransition(x, y, th, fi, current_switch_time);
                    
%                         disp(["Line",  x, y, th, fi]);
                    else
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, current_switch_time);
                    
%                         disp(["Circle",  x, y, th, fi]);
                    end
                end
                i = i + 1;
            end
            
        end
        
        function [x, y, th, fi] = buildPathAnalyticallyAuto(obj, duration_intervals, control_vector, x, y, th, fi)
            
            i = 1;
            
            for c = control_vector
                current_switch_time = duration_intervals(i);
                turn_sign = c;

                if c~=0
                    if i==2 || i==5
                        current_switch_time = abs(fi)/obj.maxSteeringVelocity;
                    end
                    
                    [x, y, th, fi, time_interval] = ...
                    obj.getKlothoideTransition(x, y, th, fi, turn_sign, current_switch_time);
                    
                    if current_switch_time>time_interval
                        residual_time_interval = current_switch_time-time_interval;
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, residual_time_interval);
                    end
                else
                    if abs(fi)<deg2rad(0.05)
                        [x, y, th, fi, not_used] = ...
                        obj.getLineTransition(x, y, th, fi, current_switch_time);
                    else
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, current_switch_time);
                    end
                end
                i = i + 1;
            end
            
        end
        
        %% Transition evaluations
        function [x1, y1, th1, fi1, time_interval] = getLineTransition(obj, x0, y0, th0, fi0, time_interval)

            v = obj.velocity;

            fi1 = fi0;
            th1 = th0;

            x1 = x0 + v*cos(th0)*time_interval;
            y1 = y0 + v*sin(th0)*time_interval;

        end
        
        function [x1, y1, th1, fi1, time_interval] = getCircleTransition(obj, x0, y0, th0, fi0, time_interval)

            l = obj.wheelBase;
            v = obj.velocity;

            fi1 = fi0;
            th1 = th0 + v/l*fi0*time_interval;
            x1 = x0 + l/fi0*( sin( (time_interval*fi0*v + l*th0)/l ) - sin(th0) );
            y1 = y0 - l/fi0*( cos( (time_interval*fi0*v + l*th0)/l ) - cos(th0) );


        end
        
        function [x1, y1, th1, fi1, time_interval] = getKlothoideTransition(obj, x0, y0, th0, fi0, turn_sign, time_interval)

            v = obj.velocity;
            l = obj.wheelBase;

            max_fi = obj.maxSteeringAngle;
            min_fi = -obj.maxSteeringAngle;

            if turn_sign>0
                max_time_interval = (max_fi - fi0)/turn_sign;
            else
                max_time_interval = (min_fi - fi0)/turn_sign;
            end

            if time_interval > max_time_interval
                time_interval = max_time_interval;
            end

            fi1 = fi0 + turn_sign*time_interval;
            th1 = th0 + v/l*fi0*time_interval + v/l*turn_sign*time_interval^2/2;
            
            % Polynom aproximation
            x1 = x0 + obj.frmCos(time_interval, th0, fi0, turn_sign);
            y1 = y0 + obj.frmSin(time_interval, th0, fi0, turn_sign);
            
            % Numerical solution
%             fCos = @(x) v*cos(th0+v/l*(fi0*x+turn_sign*x.^2/2));
%             fSin = @(x) v*sin(th0+v/l*(fi0*x+turn_sign*x.^2/2));
% 
%             x1 = x0 + integral(fCos, 0, time_interval);
%             y1 = y0 + integral(fSin, 0, time_interval);
             
        end
        
        %% Clothoid poly aproximation
        function [val] = frmCos(obj, t, th0, fi0, turn_sign)

            u = turn_sign;
            v = obj.velocity;
            l = obj.wheelBase;
            T = t;

            val = T*v*cos(th0) + (T^3*v^3*cos(th0)*(35*T^6*u^4*v^2 + 315*T^5*fi0*u^3*v^2 + 1080*T^4*fi0^2*u^2*v^2 + 1680*T^3*fi0^3*u*v^2 + 1008*T^2*fi0^4*v^2 - 3024*T^2*l^2*u^2 - 15120*T*fi0*l^2*u - 20160*fi0^2*l^2))/(120960*l^4) + ...
                (T^2*v^2*sin(th0)*(5*T^5*u^3*v^2 + 35*T^4*fi0*u^2*v^2 + 84*T^3*fi0^2*u*v^2 + 70*T^2*fi0^3*v^2 - 280*T*l^2*u - 840*fi0*l^2))/(1680*l^3);

        end
        
        function [val] = frmSin(obj, t, th0, fi0, turn_sign)

            u = turn_sign;
            v = obj.velocity;
            l = obj.wheelBase;
            T = t;

            val = T*v*sin(th0) - (T^2*v^2*cos(th0)*(5*T^5*u^3*v^2 + 35*T^4*fi0*u^2*v^2 + 84*T^3*fi0^2*u*v^2 + 70*T^2*fi0^3*v^2 - 280*T*l^2*u - 840*fi0*l^2))/(1680*l^3) + ...
                (T^3*v^3*sin(th0)*(35*T^6*u^4*v^2 + 315*T^5*fi0*u^3*v^2 + 1080*T^4*fi0^2*u^2*v^2 + 1680*T^3*fi0^3*u*v^2 + 1008*T^2*fi0^4*v^2 - 3024*T^2*l^2*u^2 - 15120*T*fi0*l^2*u - 20160*fi0^2*l^2))/(120960*l^4);

        end
        
        %% Get path numiracly
        function [x, y, th, fi] = buildPathNumeracly(obj, switch_times, control_vector, x, y, th, fi)

            % BUG: double multiply on maxSteeringVelocity
            %control_vector = control_vector * obj.maxSteeringVelocity;
            
            state = [x; y; th; fi];

            stepDivider = obj.pathTimeStepDivider;

            rightPart = @obj.bicycleODE;

            table_of_switch = control_vector;

            % time intervals to switch times
            for i=1:length(switch_times)-1
                switch_times(i+1) = switch_times(i+1) + switch_times(i);
            end
            
            switch_fnc_params = switch_times;

            u = table_of_switch(1);
            h = switch_fnc_params(1)/stepDivider;
            t = 0;
            
                while t<=switch_times(end)

                    % get control and time interval from time-controls arrays
                    if(length(switch_fnc_params)>0 && t>switch_fnc_params(1))

                        switch_fnc_params(1) = [];
                        table_of_switch(1) = [];

                        if length(table_of_switch) == 0
                            break;
                        end

                        u = table_of_switch(1);
                        h = switch_fnc_params(1)/stepDivider;
%                         disp(h);
%                         disp([x, y, th, fi]);
                    end

                    % contain data
                    [x, y, th, fi] = obj.state2coordinates(state);
                    obj.pos_x = [obj.pos_x; x];
                    obj.pos_y = [obj.pos_y; y];
                    obj.fi_arr = [obj.fi_arr; fi];
                    obj.t_arr = [obj.t_arr; t];
                    obj.th_arr = [obj.th_arr; th];
                    obj.u_arr = [obj.u_arr; u];

                    obj.x_dir_arr = [obj.x_dir_arr; cos(th+fi)];
                    obj.y_dir_arr = [obj.y_dir_arr; sin(th+fi)];

                    % solve integration step by RK4 method
                    state = obj.RK4SolveStep(rightPart, state, h, u);
                    t = t + h;
                    state = obj.stateConstrain(state);

                end
                
                %TODO: flag to show plot
                hold on
                plot(obj.pos_x, obj.pos_y, 'LineWidth', 2.1); grid on; grid minor; axis equal
                
                if obj.isDrawArrows == 1
                    quiver(obj.pos_x(1:50:end), obj.pos_y(1:50:end),...
                        obj.x_dir_arr(1:50:end), obj.y_dir_arr(1:50:end), 0.3)
                end
%               disp([x, y, th, fi]);

% %               hold on;
% %               plot(obj.t_arr, obj.fi_arr, 'LineWidth', 2); grid on; grid minor
% %               plot(obj.t_arr, obj.pos_x, 'LineWidth', 2); grid on; grid minor
% %               plot(obj.t_arr, obj.pos_y, 'LineWidth', 2); grid on; grid minor
% %               plot(obj.t_arr, obj.th_arr, 'LineWidth', 2); grid on; grid minor
% %               legend('fi','x', 'y', 'th')
% %               hold off;
        end
            
        function [x_new] = RK4SolveStep(obj, rightPart, x, h, u)
            
            k1 = rightPart(x, u);
            k2 = rightPart(x+k1*h/2, u);
            k3 = rightPart(x+k2*h/2, u);
            k4 = rightPart(x+k3*h, u);

            x_new = x + h/6*(k1+2*k2+2*k3+k4);

        end
        
        function [dState] = bicycleODE(obj, state, control)
        %BICYCLEODE 
        
            l = obj.wheelBase;
            v = obj.velocity;

            max_u = obj.maxSteeringVelocity;
            max_fi = obj.maxSteeringAngle;

            u = control(1);

            [x, y, th, fi] = obj.state2coordinates(state);

            u  = obj.constrain(u, max_u, -max_u); 
            fi = obj.constrain(fi, max_fi, -max_fi);

            dState(1, 1) = v * cos(th);
            dState(2, 1) = v * sin(th);
            dState(3, 1) = v/l * (fi); % TODO: to be precise tan(fi)
            dState(4, 1) = u;

        end
        
        function [constranedState] = stateConstrain(obj, state)

            [x, y, th, fi] = obj.state2coordinates(state);
            max_fi = obj.maxSteeringAngle;
            fi = obj.constrain(fi, max_fi, -max_fi);
            constranedState = [x; y; th; fi];

        end
        
        function [x, y, th, fi] = state2coordinates(obj, state)

            x = state(1);
            y = state(2);
            th = state(3);
            fi = state(4);

        end

        function [outVal] = constrain(obj, inVal, maxVal, minVal)
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
        
        %% Get initial control approximation from Dubins
        function [t,u] = initFromTable(obj, times, control)

            w_max = obj.maxSteeringVelocity;
            fi0 = obj.initSteeringAngle;
            
            L = obj.maxSteeringAngle;
            R = -obj.maxSteeringAngle;
            S = 0;

            T1 = times(1);
            T2 = times(2);
            T3 = times(3);

            T_s = abs((R-S)/obj.maxSteeringVelocity);
            T_rl = abs((R-L)/obj.maxSteeringVelocity);


                if  isequal(control, [1 0 -1]) %LSR
                    T0 = abs((L-fi0)/w_max);
                    u = [1, -1, 0, -1, 1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [-1 0 -1]) %RSR
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 0, -1, 1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [-1 1 -1]) %RLR
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 1, -1, -1];
                    t = [T1+T0, 0, T2+T_rl, T3+T_rl, T_s];
                    return;
                end


                if  isequal(control, [1 -1 1]) %LRL
                    T0 = abs((L-fi0)/w_max);
                    u = [1, -1, -1, 1, -1];
                    t = [T1+T0, 0, T2+T_rl, T3+T_rl, T_s];
                    return;
                end

                if  isequal(control, [-1 0 1]) %RSL
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 0, 1, -1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [1 0 1]) %LSL
                    T0 = abs((R-fi0)/w_max);
                    u = [1,-1, 0, 1, -1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end
        end
        
        function [times, control] = getApproximationFromDubins(obj, x1)

            dubConnObj = dubinsConnection;
            dubConnObj.MinTurningRadius = obj.wheelBase/obj.maxSteeringAngle;
            startPose = [obj.initXPos obj.initYPos obj.initHeading];
            goalPose = [x1 obj.targetYPos 0];
            [pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
            
            if obj.isDrawDubins == 1
                show(pathSegObj{1})
            end
            
            control = [];
            
            for type=pathSegObj{1}.MotionTypes
                if type{1} == 'L'
                    control = [control 1];
                end
                if type{1} == 'S'
                    control = [control 0];
                end
                if type{1} == 'R'
                    control = [control -1];    
                end
            end

            times = pathSegObj{1}.MotionLengths/obj.velocity;

        end

        %% Test analytical VS numerical
        function [res, initStates, T, C] = test(obj, testSize)


            initStates = [];
            res = [];
            T = [];
            C = [];

            for i=1:testSize
                
                x = rand; y = rand; th = rand*pi; fi = 0.3*rand;
                
                clc; disp(i);
                % random control
                control_vector = randi([-1 1], 1, 5);

                % random time intervals
                a = 0;  
                b = 10;
                r = (b-a).*rand(5,1) + a;
                switch_times = r';

                % manual [t c]
%                 switch_times = [ 9.7635    3.3120    6.2371    5.2706    4.5908];
%                 control_vector = [0     0     1    -1     1];
%                 x = 0.8082; y = 0.2797; th = 2.0891; fi = 0.0088;
                
                
                [x_a, y_a, th_a, fi_a] = obj.buildPathAnalytically(switch_times, control_vector, x, y, th, fi);
                [x_n, y_n, th_n, fi_n] =    obj.buildPathNumeracly(switch_times, control_vector, x, y, th, fi);

                T = [T; switch_times];
                C = [C; control_vector];
                res = [res; [x_a-x_n, y_a-y_n ,th_a-th_n, fi_a-fi_n]];
                initStates = [initStates; [x, y, th, fi]];
                
%                 break;
            end

        end
    end
end

