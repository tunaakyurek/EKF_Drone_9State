%% test_fixed_logging.m - Test the fixed logging system with realistic data types

clear; clc; close all;

fprintf('Testing Fixed Enhanced EKF Logging System...\n');

try
    % Initialize logger
    logger = enhanced_ekf_logger('test_fixed_log.txt');
    
    % Test with realistic system status data (like in main_enhanced_ekf.m)
    system_status = struct();
    system_status.initial_position = [0, 0, 0];  % numeric array
    system_status.initial_attitude = [0, 0, 0];  % numeric array
    system_status.flight_mode = 'stabilized';    % string
    system_status.sensor_health = struct('gps', true, 'baro', true, 'mag', true, 'imu', true);  % struct
    
    fprintf('Testing system status logging with mixed data types...\n');
    logger.log_system_status(system_status);
    
    % Test other logging functions
    logger.log('INFO', 'Testing mixed data type logging');
    logger.log_flight_mode('stabilized', 0.0, 'System initialization');
    
    % Close logger
    logger.close();
    
    fprintf('✅ Fixed logging system test completed successfully!\n');
    fprintf('📁 Test log file created: test_fixed_log.txt\n');
    
    % Display the log content to verify it works
    fprintf('\n📝 Fixed log content:\n');
    fprintf('=====================\n');
    fid = fopen('test_fixed_log.txt', 'r');
    if fid ~= -1
        content = fread(fid, '*char')';
        fclose(fid);
        fprintf('%s\n', content);
    end
    fprintf('=====================\n');
    
    fprintf('\n✅ Enhanced EKF logging system is now ready for full simulation!\n');
    
catch ME
    fprintf('❌ Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('Error in: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end