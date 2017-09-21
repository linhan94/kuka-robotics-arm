function [translation_error, rotation_error] = calculate_pose_error(estimated_traj, desired_traj)

    [M, N] = size(desired_traj);
    %For rotation error, you have to calculate relative error between two
    %rotation matrices and use rodrigues function to compute the error
    %in term of 3x1 vector. 
    %The template of rodrigues function is given below
    %R = rodrigues(r) and r = rodrigues(R), where
    %r: 3x1 vector (axis of rotation with the norm equal to angle rotation)
    %R: 3x3 rotation matrix
    if (N == 2)
        %Implement code for calculating pose error in 4R-planar robot here
        translation_error = randn(1, M);
        rotation_error = randn(1, M);
    else
        %Implement code for calculating pose error in youbot and KUKA here
        translation_error = randn(1, M);
        rotation_error = randn(1, M);
    end

end