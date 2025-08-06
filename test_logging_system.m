%% test_logging_system.m - Quick test of the enhanced logging system
% Tests the logging functionality before running the full simulation

clear; clc; close all;

fprintf('Testing Enhanced EKF Logging System...\n');

try
    % Initialize logger
    logger = enhanced_ekf_logger('test_log.txt');
    
    % Test basic logging
    logger.log('INFO', 'Testing logging system initialization');
    logger.log('WARNING', 'This is a test warning message');
    
    % Test system status logging
    test_status = struct();
    test_status.test_mode = 'active';
    test_status.sensors = [1, 1, 1, 1];
    logger.log_system_status(test_status);
    
    % Test flight mode logging
    logger.log_flight_mode('test_mode', 1.5, 'System test');
    
    % Test sensor update logging
    logger.log_sensor_update('GPS', 2.0, [10, 20, -5], true);
    logger.log_sensor_update('IMU', 2.1, [0.1, 0.2, 9.8, 0.01, 0.02, 0.03], true);
    
    % Test performance logging
    x_true_test = [10; 20; -5; 1; 0.5; -0.2; 0.1; 0.05; 0.78];
    x_est_test = [10.1; 19.9; -4.9; 1.1; 0.4; -0.25; 0.12; 0.04; 0.8];
    u_test = [15.2; 0.1; 0.05; 0.02];
    logger.log_performance(3.0, x_true_test, x_est_test, u_test);
    
    % Test emergency logging
    logger.log_emergency('LOW_BATTERY', 4.0, 'Battery voltage below 10.5V');
    
    % Test EKF metrics logging
    innovation_test = [0.1; 0.05; 0.02];
    covariance_test = 0.1 * eye(9);
    logger.log_ekf_metrics(5.0, innovation_test, covariance_test, 1.2);
    
    % Close logger and generate summary
    logger.close();
    
    fprintf('✅ Logging system test completed successfully!\n');
    fprintf('📁 Test log file created: test_log.txt\n');
    fprintf('🔍 Check the log file to verify all logging functions work correctly.\n\n');
    
    % Display a sample of the log content
    fprintf('📝 Sample log content:\n');
    fprintf('===================\n');
    fid = fopen('test_log.txt', 'r');
    if fid ~= -1
        % Read first 20 lines
        for i = 1:20
            line = fgetl(fid);
            if ischar(line)
                fprintf('%s\n', line);
            else
                break;
            end
        end
        fclose(fid);
        fprintf('===================\n');
    end
    
    fprintf('\n✅ Enhanced EKF logging system is ready for full simulation!\n');
    
catch ME
    fprintf('❌ Error testing logging system: %s\n', ME.message);
    fprintf('Error in: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
end