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
        PARAM

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
            
            
        end
        
        
        %% NEWTON
      
        function [Y] = funcOfResiduals(obj, X)
            % 2, 5 -auto
            X = abs(X);
            times = [obj.PARAM 0 X(1) X(2) 0];
            [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(times, obj.controlVector,...
            obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
            
            Y = [y_a, sin(0.5*th_a)];
        end
        
        
        %% Find path
        function [] = findPath(obj)
        
            %% TODO: initial signs ???
            control0 = -[1, -1, 0, -1, 1]; % initial signs of turns
            times = [0, 0, 0, 0, 0];
            
            control = obj.maxSteeringVelocity*control0;
            obj.controlVector = control;
            
            
            %% 
            tic
            
            times_1 = 0:0.01:20; 
            
            Tsum_from_t1 = ones(size(times_1))*200;
            T_st_array = ones(size(times_1));
            T_1_array = ones(size(times_1));
            T_2_array = ones(size(times_1));
            
            %% Two cases of first turn signs [1, -1, ...] or [-1, 1, ...]
            %% Three cases of final heading -2pi, 0, +2pi 
            
            i = 0;
            for t1 = times_1
            
                i=i+1;
                
                [TSUM, t1_, t2, tst, cf] = obj.getIntervalTimes(t1, control0);
                % вот тут ... 
                
                if cf==1
                    continue;
                end
                
                Tsum_from_t1(i) = TSUM;
                T_st_array(i) = tst;
                T_1_array(i) = t1;
                T_2_array(i) = t2;
            end
            
            return;
%             
%             %% fi_a - bad 0.34((
% 
%             [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAutoREC(times, obj.controlVector,...
%              obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle);
% 
%             %% figure
%              global gStates
%                          
%              
%              plot(gStates(:,1), gStates(:,2), '.', "LineWidth", 2);
%              grid on; grid minor;
%              axis equal
% 
%              %% return final 
%              ret_x = x_a;
%              ret_y = y_a;
%              ret_th = th_a;
%              ret_fi = fi_a;

             toc
        end

        
        %% NEW ALGORITHM
        function [TSUM, t1, t2, tst, cf] = getIntervalTimes(obj, t1, control0)
            
            tst = 0;
            TSUM = 200;
            cf = 0;

            times = zeros(1, 5); 
            times(1) = t1;

            %% TODO: if t_i == 0, pass part of motion 
            %% Get th_a 
            [x_a, y_a, th_a, fi_a, x1, y1, th1] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );

            t0 = obj.maxSteeringAngle/obj.maxSteeringVelocity;

            %% Find last control sign
            control0(4) = abs(control0(4))*sign(th1);
            control0(5) = -control0(4);

            %% In constrains

            t2 = sqrt(abs(th1/(obj.velocity/obj.wheelBase*obj.maxSteeringVelocity)));

            %% Out constrains 
            if t2>t0
                t2 = abs(th1/(obj.velocity/obj.wheelBase*obj.maxSteeringAngle));
            end

            if t2 < 0
                cf = 1;
                return;
            end
            %% TODO: Check 2pi or -2pi    
            %% Check t2
            %% If bad one of times pass and set NaN

            %% Get residuasl for y
            times(4) = t2;
            [x_a, y_a, th_a, fi_a, x1, y1, th1] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );

            %% Get Tst from y residual and th1 - final heading for first motion interval
            if abs(y_a)<0.0
                tst = 0;
            else
                tst = -y_a/(obj.velocity*sin(th1));
            end
            times(3) = tst;

            if tst < 0 || tst > 100
                cf = 1;
                return;
            end

            %% Final result
            [x_a, y_a, th_a, fi_a, x1, y1, th1, ta2, ta5] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );

           [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );
                
            TSUM = tst+t1+t2+ta2+ta5;                
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
        
        function [x, y, th, fi, xk1, yk1, thk1, ta2, ta5] = buildPathAnalyticallyAuto(obj, duration_intervals, control_vector, x, y, th, fi)
            duration_intervals = abs(duration_intervals);
            i = 1;
            
            for c = control_vector
                current_switch_time = duration_intervals(i);
                turn_sign = c;

                if c~=0
                    if i==2 || i==5
                        current_switch_time = abs(fi)/obj.maxSteeringVelocity;
                        if i == 5 || i == 2
                            turn_sign = -sign(fi)*abs(turn_sign);
                        end
                        
                        if i==2
                            ta2 = current_switch_time;
                        end
                        if i==5
                            ta5 = current_switch_time;
                        end
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
                
                if i == 3
                    xk1 = x; yk1 = y; thk1 = th;
                end
                
                i = i + 1;
            end
            
        end
        
        function [x, y, th, fi] = buildPathAnalyticallyAutoREC(obj, duration_intervals, control_vector, x, y, th, fi)
            
            i = 1;
            duration_intervals = abs(duration_intervals);
            
            global gStates;
            
            h = 0.05;
            
            for c = control_vector
                current_switch_time = duration_intervals(i);
                turn_sign = c;
                
                if c~=0
                    if i==2 || i==5
                        current_switch_time = abs(fi)/obj.maxSteeringVelocity;
                        if i == 5 || i == 2
                            turn_sign = -sign(fi)*abs(turn_sign);
                        end
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
        
        
        %% Utils
        function has_diff_signs = check_signs(obj, matrix)
            % Проверка наличия положительных и отрицательных элементов
            has_positive = any(matrix(:) > 0);
            has_negative = any(matrix(:) < 0);

            % Если есть и положительные, и отрицательные элементы -> true
            has_diff_signs = has_positive && has_negative;
        end

        function intersection_point = plane_intersection_z0(obj, P1, P2, P3)
            % Проверяем размерности входных данных
            P1 = P1(:)'; % Преобразуем в строку [x1, y1, z1]
            P2 = P2(:)';
            P3 = P3(:)';

            % Находим самую нижнюю точку (с минимальным z)
            points = [P1; P2; P3];
            [~, min_idx] = min(points(:, 3));
            min_z_point = points(min_idx, :);

            % Вычисляем два вектора в плоскости
            v1 = P2 - P1;
            v2 = P3 - P1;

            % Вычисляем нормаль к плоскости
            n = cross(v1, v2);

            % Проверяем, что векторы не коллинеарны
            if norm(n) < 1e-10
                error('Точки коллинеарны. Плоскость не определена.');
            end

            % Проверяем, горизонтальна ли плоскость (нормаль параллельна оси Z)
            if abs(n(1)) < 1e-10 && abs(n(2)) < 1e-10
                % Плоскость горизонтальна
                if abs(min_z_point(3)) < 1e-10
                    % Плоскость проходит через z=0
                    intersection_point = [min_z_point(1), min_z_point(2), 0];
                else
                    % Плоскость не пересекает z=0
                    intersection_point = [NaN, NaN, NaN];
                end
                return;
            end

            % Вычисляем d для уравнения плоскости: n_x*x + n_y*y + n_z*z = d
            d = dot(n, P1);

            % Уравнение линии пересечения при z=0: n_x*x + n_y*y = d
            % Проекция точки (x0, y0) на эту линию в 2D
            x0 = min_z_point(1);
            y0 = min_z_point(2);
            t = (n(1)*x0 + n(2)*y0 - d) / (n(1)^2 + n(2)^2);
            x_proj = x0 - t * n(1);
            y_proj = y0 - t * n(2);

            intersection_point = [x_proj, y_proj, 0];
        end
    end
    
    methods (Static)
             function y = fast_sin(x)
                % Приведение угла к диапазону [0, 2π)
                x = mod(x, 2*pi);

                % Определение знака и приведение к диапазону [0, π]
                sign = 1;
                if x > pi
                    x = x - pi;
                    sign = -1;
                end

                % Приведение к диапазону [0, π/2]
                if x > pi/2
                    x = pi - x;
                end

                % Аппроксимация рядом Тейлора (схема Горнера)
                x2 = x .* x;
                y = x .* (1 - x2/6 .* (1 - x2/20 .* (1 - x2/42)));

                % Учет знака
                y = sign .* y;
            end

            function y = fast_cos(x)
                % Приведение угла к диапазону [0, 2π)
                x = mod(x, 2*pi);

                % Определение знака и приведение к диапазону [0, π]
                sign = 1;
                if x > pi
                    x = 2*pi - x;
                    sign = -1;
                end

                % Приведение к диапазону [0, π/2] и корректировка знака
                if x > pi/2
                    x = pi - x;
                    sign = -sign;
                end

                % Аппроксимация рядом Тейлора (схема Горнера)
                x2 = x .* x;
                y = 1 - x2/2 .* (1 - x2/12 .* (1 - x2/30));

                % Учет знака
                y = sign .* y;
            end
        end
end

