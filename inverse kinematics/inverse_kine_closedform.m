function theta = inverse_kine_closedform(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    if (strcmp(option, 'q4'))
        theta = zeros(M, 5);
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end