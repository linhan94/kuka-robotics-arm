function [joint_profile, velocity_profile] = create_trajectory_1bb(data)
%% Variables initialization and Prellocation
    P = size(data,3);
    time_set = 0:3:12;
    dt = 0.05;
    period = 3/dt;
    size_Mat = (3/dt)*(P-1)+1;
    
    velocity_profile = zeros(size_Mat, 5); 
    
    translationPos = zeros([3 size_Mat]);
    rotationPos = zeros([3 3 size_Mat]);
    slerp = zeros([size_Mat+1,4]);

%% Linear interpolation in Cartesian space of translation matrix
    for i = 1:P-1
        set = [0 period period*2 period*3];
        t0 = time_set(i);
        tf = time_set(i+1);
        Pos_i = data(1:3,4,i);
        Pos_f = data(1:3,4,i+1);
        for t = t0:dt:tf
            set = set+1;
            translationPos(:,set(i)) = Pos_i + (Pos_f-Pos_i)*(t-t0)/(tf-t0);
        end
    end
    translationPos = reshape(translationPos, [3,1,size_Mat]);
    
%% SLERP in cartesian space of rotation matrix
    for i = 1:P-1
        t0 = time_set(i);
        tf = time_set(i+1);
        set = [0 period period*2 period*3];
        Rot_i = rotm2quat(data(1:3,1:3,i));
        Rot_f = rotm2quat(data(1:3,1:3,i+1));
        if dot(Rot_i,Rot_f) >= 0
            theta = acos(dot(Rot_i,Rot_f))
        else
            Rot_f = -Rot_f;
            theta = acos(-dot(Rot_i,Rot_f));
        end
        for t = t0:dt:tf
            set = set + 1;
            normalized_time = (t-t0)/(tf-t0);
            slerp(set(i),:) = sin((1-normalized_time)*theta)*Rot_i/sin(theta)...
                + sin((normalized_time)* theta)*Rot_f/sin(theta);
            rotationPos(:,:,set(i)) = quat2rotm(slerp(set(i),:));
        end
        
        split_num = 25;
        qm = zeros(split_num,4);
        for k=1:split_num
            qm(k,:) = (Rot_i*sin((1-(k-1)/split_num)*theta)+Rot_f*sin(theta*(k-1)/split_num))/sin(theta);
        end
    end
    save('slerp.mat', 'slerp');
%% Interpolated transformation matrix
    transMat = [rotationPos translationPos; ...
        zeros(1,3,size(translationPos,3)) ones(1,1,size(translationPos,3))];
    save('transMat.mat', 'transMat');
%% Closed form inverse kinematics
    joint_profile = ikine_closedform(transMat);
    for i = 2:size_Mat
        velocity_profile(i,:) = (joint_profile(i,:)-joint_profile(i-1,:))/dt;
    end
end