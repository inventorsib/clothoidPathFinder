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
            
            
            %% FIRST 
            
             if obj.isDrawFirstNumerical == 1
                
                % new idea 
                times = [times(1) times(3) times(4)];
                st = times;
                t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
                times = [st(1) t0 st(2) st(3) t0];
                
                obj.buildPathAnalyticallyAutoREC(times,  obj.controlVector,...
                    obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
                
                global gStates
                plot(gStates(:,1), gStates(:,2), '.', "LineWidth", 2);
                grid on; grid minor;
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
            dubins_times = times;
            
            
            tic
% %             for i = 1:1000
% %                disp(i)
% %                 
% %                times = rand([1, 3])*10;
% %                times(2) = 0;
% %                
% %                if i == 0
% %                    times = dubins_times;
% %                end
% %                
% %                c = 1;
% %                if i > 0
% %                 c = sign(randn);
% %                end
% %                obj.controlVector = c*obj.controlVector;
% %                
% %                for p=100
% %                   
% %                   obj.p_parameter = p; 
% %                   st = fminunc(@obj.func, times);
% %                   times = [st(1) 0 st(2) st(3) 0];
% %                   
% %                end
% %                
% %                [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(times, obj.controlVector,...
% %                 obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
% %             
% %                 if abs(y_a)+abs(sin(0.5*th_a))<0.01
% %                     [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAutoREC(times, obj.controlVector,...
% %                     obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
% %                     break;
% %                 end
% %                % times = fminsearch(@obj.func, times);
% %             end
            toc
            clf
            A = zeros(100);
            B = zeros(100);
            i = 1;
            for t1=0:0.1:10
                j = 1;
                for t2 = 0:0.1:10
                    times = [t1 0 0.0 t2 0];
                    [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(times, control,...
                    obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
                    A(i, j) = y_a;
                    B(i, j) = th_a;
                    
                    if(abs(y_a)+abs(th_a)<0.1)
                        disp([t1 t2])
                    end
                    j = j + 1;
                end
                i = i + 1;
            end
            
            clf
%             imshow(sign(A).*sign(B), [])
            surf(A, 'EdgeColor','none')
            hold on
            surf(B, 'EdgeColor','none')
            
            % new idea 
            st = times;
            t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
            times = [st(1) t0 st(2) st(3) t0];
            
%             [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAutoREC(times, control,...
%                 obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
            
             global gStates
             plot(gStates(:,1), gStates(:,2), '.', "LineWidth", 2);
             grid on; grid minor;
            
        end

        %% \\
        function [J] = func(obj, switch_times)
            switch_times = abs(switch_times);
            % new idea 
            st = switch_times;
            t0 = abs(obj.maxSteeringAngle/obj.maxSteeringVelocity);
            switch_times = [st(1) t0 st(2) st(3) t0];
            
            p = obj.p_parameter;

            x0 = obj.initXPos;
            y0 = obj.initYPos;
            th0 = obj.initHeading;
            fi0 = obj.initSteeringAngle;
            
           
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(...
                switch_times,...
                obj.controlVector, ...
                x0, y0, th0, fi0);
            
            y_target = obj.targetYPos;

            swtc = abs(min(0, switch_times));

            % TODO: задать и обосновать коэффициент
            J = p*(1*(y_a-y_target)^2 + 10*abs(sin(th_a))^2)^0.5 ...
                + 10*norm(swtc)*norm(swtc)...
                + 10*sum(abs(switch_times)); 
            
%             disp([(y_a-y_target), 10*abs(sin(0.5*th_a))])
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
            duration_intervals = abs(duration_intervals);
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
            duration_intervals = abs(duration_intervals);
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
        
        function [x, y, th, fi] = buildPathAnalyticallyAutoREC(obj, duration_intervals, control_vector, x, y, th, fi)
            
            i = 1;
            duration_intervals = abs(duration_intervals);
            
            global gStates;
            
            h = 0.001;
            
            for c = control_vector
                current_switch_time = duration_intervals(i);
                turn_sign = c;
                
                if c~=0
                    if i==2 || i==5
                        current_switch_time = abs(fi)/obj.maxSteeringVelocity;
                    end
                    
                    xr = x; yr = y; thr = th; fir = fi; 
                    
                    [x, y, th, fi, time_interval] = ...
                    obj.getKlothoideTransition(x, y, th, fi, turn_sign, current_switch_time);
                
                        for tr=0:h:time_interval
                            [xr, yr, thr, fir, not_used] = ...
                            obj.getKlothoideTransition(xr, yr, thr, fir, turn_sign, h);
                            gStates = [gStates; [xr, yr, thr, fir]];
                        end
                    
                    if current_switch_time>time_interval
                        
                        xr = x; yr = y; thr = th; fir = fi; 
                        
                        residual_time_interval = current_switch_time-time_interval;
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, residual_time_interval);
                    
                        for tr=0:h:residual_time_interval
                            [xr, yr, thr, fir, not_used] = ...
                            obj.getCircleTransition(xr, yr, thr, fir, h);
                            gStates = [gStates; [xr, yr, thr, fir]];
                        end
                    end
                else
                    if abs(fi)<deg2rad(0.005)
                        xr = x; yr = y; thr = th; fir = fi; 
                        [x, y, th, fi, not_used] = ...
                        obj.getLineTransition(x, y, th, fi, current_switch_time);
                    
                        for tr=0:h:current_switch_time
                            [xr, yr, thr, fir, not_used] = ...
                            obj.getLineTransition(xr, yr, thr, fir, h);
                            gStates = [gStates; [xr, yr, thr, fir]];
                        end
                    else
                        xr = x; yr = y; thr = th; fir = fi; 
                        
                        [x, y, th, fi, not_used] = ...
                        obj.getCircleTransition(x, y, th, fi, current_switch_time);
                    
                        for tr=0:h:current_switch_time
                            [xr, yr, thr, fir, not_used] = ...
                            obj.getCircleTransition(xr, yr, thr, fir, h);
                            gStates = [gStates; [xr, yr, thr, fir]];
                        end
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
                    disp("LSR")
                    T0 = abs((L-fi0)/w_max);
                    u = [1, -1, 0, -1, 1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [-1 0 -1]) %RSR
                    disp("RSR")
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 0, -1, 1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [-1 1 -1]) %RLR
                    disp("RLR")
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 1, -1, -1];
                    t = [T1+T0, T_s, T2+T_rl, T3+T_rl, T_s];
                    return;
                end


                if  isequal(control, [1 -1 1]) %LRL
                    disp("LRL")
                    T0 = abs((L-fi0)/w_max);
                    u = [1, -1, -1, 1, -1];
                    t = [T1+T0, T_s, T2+T_rl, T3+T_rl, T_s];
                    return;
                end

                if  isequal(control, [-1 0 1]) %RSL
                    disp("RSL")
                    T0 = abs((R-fi0)/w_max);
                    u = [-1, 1, 0, 1, -1];
                    t = [T1+T0, T_s, T2, T_s+T3, T_s];
                    return;
                end

                if  isequal(control, [1 0 1]) %LSL
                    disp("LSL")
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


    end
end

