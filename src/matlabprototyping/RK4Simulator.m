classdef RK4Simulator
    properties
        y       % State variable
        t       % Time variable
        dt      % Time step (1/50 sec for 50 FPS)
        equation % Function handle for the ODE
        running % Control flag
    end
    
    methods
        % Constructor
        function obj = RK4Simulator()
            obj.y = 1.0; % Initial condition
            obj.t = 0.0;
            obj.dt = 1/50; % 50 FPS
            obj.equation = @RK4Simulator.defaultEquation; % Default ODE
            obj.running = true;
        end

        % Default ODE function
        function dydt = defaultEquation(~, y, t)
            dydt = -y; % Example: dy/dt = -y
        end
        
        % RK4 Integration Step
        function y_new = RK4Step(obj)
            f = obj.equation; % Get current ODE function
            dt = obj.dt;
            
            k1 = dt * f(obj, obj.y, obj.t);
            k2 = dt * f(obj, obj.y + 0.5 * k1, obj.t + 0.5 * dt);
            k3 = dt * f(obj, obj.y + 0.5 * k2, obj.t + 0.5 * dt);
            k4 = dt * f(obj, obj.y + k3, obj.t + dt);
            
            y_new = obj.y + (k1 + 2*k2 + 2*k3 + k4) / 6;
        end
        
        % Update Equation Function
        function obj = updateEquation(obj, newEq)
            obj.equation = newEq; % Replace with new function
        end

        % Main Simulation Loop
        function run(obj)
            while obj.running
                tic; % Start timing
                
                obj.y = obj.RK4Step(); % Runge-Kutta Step
                obj.t = obj.t + obj.dt;
                
                fprintf('Time: %.2f, State: %.4f\n', obj.t, obj.y);
                
                elapsed = toc; % Time taken for iteration
                pause(max(0, obj.dt - elapsed)); % Maintain 50 FPS
            end
        end
    end
end