% Run all test scripts sequentially

close all; clear; clc;

ts = dir('test_*.m');
for i = 1:size(ts, 1)
    run_test(ts(i).name);
    
    disp('Press a key !')
    pause();
end
disp('Finished!')

function run_test(filename)
run(filename)
end