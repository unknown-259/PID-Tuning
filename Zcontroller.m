function [T, pid_output, Z_pid_err] = Zcontroller(Z_pid, z_d, z, z_dot, dt, tau, k)
    MAX_OUT = 3;
    MAX_T = 255;
    
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p = ; % HERE TO CHANGE PORPORTIAL GAINS
    K_i = 5; % 
    K_d = ; % HERE TO CHANGE DERIVATIVE GAINS
    
    % Proportional
    Z_pid_err.z_curr_error = z_d - z;
    pid_p = K_p * Z_pid_err.z_curr_error; 
    
    % Derivative
	% ***NOTE***
    %   Use (Z_pid_err.z_curr_error - Z_pid.z_curr_error) = (measurement - prevMeasurement) once we start changing the
	%   reference signal
%     Z_pid_err.deriv = (2*K_d/(2*tau + dt))*(Z_pid_err.z_curr_error - Z_pid.z_curr_error) + ((2*tau - dt)/(2*tau + dt))*Z_pid.deriv;
    Z_pid_err.deriv = -K_d*z_dot;
    pid_d = Z_pid_err.deriv;
    
    % Integral
%     Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + K_i*0.5*dt*(Z_pid_err.z_curr_error + Z_pid.z_curr_error);
    Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + K_i*dt*(Z_pid_err.z_curr_error);
    pid_i = Z_pid_err.z_cumm_error;
    
    % Integral saturations
%     if(pid_i > MAX_OUT)
%         pid_i = MAX_OUT;
%     end
%     if(pid_i < -MAX_OUT)
%         pid_i = -MAX_OUT;
%     end
    
    % Store previous error
    Z_pid_err.z_prev = Z_pid_err.z_curr_error;
    
    % Final PID result
    pid_output = [pid_p, pid_i, pid_d];
    
    T = pid_p + pid_i + pid_d;
    T = uint8(min(max(0,T), 255));
    
    
    
    % Saturations
%     output_n = min(max(0.001, output_z), MAX_OUT); % Apply saturations from 0-3(N)
%     output_n = output_n*(MAX_T/MAX_OUT); % Convert to 0-255
%     T = uint8(output_n);
    
end