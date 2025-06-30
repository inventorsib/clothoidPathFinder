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
        
        
        
        %% Find path
        function [TSUM] = findPath(obj, c_sign, show, th_case)
        
            global gStates
            gStates = [];
            %% TODO: initial signs ???
            control0 = c_sign*[1, -1, 0, -1, 1]; % initial signs of turns
            
            control = obj.maxSteeringVelocity*control0;
            obj.controlVector = control;
            
            
            %% 
            tic
            
            times_1 = 0:0.01:100; 
            
            Tsum_from_t1 = ones(size(times_1))*200;
            T_st_array = ones(size(times_1));
            T_1_array = ones(size(times_1));
            T_2_array = ones(size(times_1));
            
            %% Two cases of first turn signs [1, -1, ...] or [-1, 1, ...]
            %% Three cases of final heading -2pi, 0, +2pi 
            
            i = 0;
            for t1 = times_1
            
                i=i+1;
                
                [TSUM, t1_, t2, tst, cf] = obj.getIntervalTimes(t1, control0, 0, th_case);
                % вот тут ... 
                
                if cf==1
                    continue;
                end
                
                Tsum_from_t1(i) = TSUM;
                T_st_array(i) = tst;
                T_1_array(i) = t1;
                T_2_array(i) = t2;
            end
            
            [mv, indx] = min(Tsum_from_t1);
            t1 = T_1_array(indx);
            disp(indx);
            [TSUM, t1_, t2, tst, cf] = obj.getIntervalTimes(t1, control0, 1, th_case);
            
            if show==1
                global gStates
                plot(gStates(:,1), gStates(:,2), '-', "LineWidth", 2);
                grid on; grid minor;
                axis equal
            end
            
            toc
        end

        
        %% NEW ALGORITHM
        function [TSUM, t1, t2, tst, cf] = getIntervalTimes(obj, t1, control0, rf, th_case)
            
            tst = 0;
            TSUM = 1000;
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

            %% In constrains
            t2 = sqrt(abs((th1+th_case)/(obj.velocity/obj.wheelBase*obj.maxSteeringVelocity)));

            %% Out constrains 
            if t2>t0
                t2 = abs((th1+th_case)/(obj.velocity/obj.wheelBase*obj.maxSteeringAngle));
            end

            %% Check t2
            if t2 < 0
                cf = 1;
                return;
            end
            %% TODO: Check 2pi or -2pi    
            

            %% Get residuasl for y
            times(4) = t2;
            [x_a, y_a, th_a, fi_a, x1, y1, th1] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );

            %% Get Tst from y residual and th1 - final heading for first motion interval
          
            if abs(sin(th1))<0.01
                cf = 1;
                return;
            end
            
            tst = -y_a/(obj.velocity*sin(th1));
            
            times(3) = abs(tst);

            if tst < 0 || tst > 100
                cf = 1;
                return;
            end

            %% Final result
            [x_a, y_a, th_a, fi_a, x1, y1, th1, ta2, ta5] = obj.buildPathAnalyticallyAuto(...
                times, obj.controlVector,...
                obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
            );

            if rf == 1
               [x_a, y_a, th_a, fi_a] = obj.buildPathAnalyticallyAutoREC(...
                    times, obj.controlVector,...
                    obj.initXPos, obj.initYPos, obj.initHeading, obj.initSteeringAngle ...
                );
            end
            
            if norm([y_a, sin(0.5*th_a), fi_a])>0.01
                cf = 1;
                return; 
            end
            
            TSUM = tst+t1+t2+ta2+ta5;                
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
        
        
    end
end

