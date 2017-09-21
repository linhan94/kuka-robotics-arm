function theta = inverse_kine_jacobian(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    
    if (strcmp(option, 'q1'))
        theta = zeros(M, 4);
    elseif (strcmp(option, 'q4'))
        theta = zeros(M, 5);
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end