function [joint_profile, velocity_profile] = create_trajectory_1c(data)
    [M, N] = size(data);
    joint_profile = zeros(100, 5);
    velocity_profile = zeros(100, 5);

    zeroPoint = zeros(1,3);
    pose = zeros(5,5);
    
    %inverse kinematic for set point
    for i = 1:5
        pose(i,:) = ikine_closedform(data(:,:,i));
    end
    
    %calculate constant velocity
    constant_veloc = zeros(5,5);
    for i = 1:4
        constant_veloc(i,:) = (pose(i+1,:)-pose(i,:))/2;
    end
    
    %get parameter
    for i = 1:4
        [a(:,:,i) b(:,:,i) c(:,:,i)] = trapezoidal_traj(pose(i,:),zeroPoint,zeroPoint,pose(i+1,:),zeroPoint,zeroPoint,constant_veloc(i,:));
    end
    
    k = 1;
    for i = 1:4
        for t = linspace(0,3,25)
            if t <= 1
                % acceleration stage
                joint_profile(k,:) = a(1,:,i) + a(2,:,i)*t + a(3,:,i)*t^2;
                velocity_profile(k,:) = a(2,:,i) + 2*a(3,:,i)*t;
            elseif t <= 2
                % constant velocity stage
                joint_profile(k,:) = b(1,:,i) + b(2,:,i)*t + b(3,:,i)*t^2;
                velocity_profile(k,:) = b(2,:,i) + 2*b(3,:,i)*t;
            else
                % deceleration stage
                joint_profile(k,:) = c(1,:,i) + c(2,:,i)*t + c(3,:,i)*t^2;
                velocity_profile(k,:) = c(2,:,i) + 2*c(3,:,i)*t;
            end
            k = k + 1;
        end
    end

    figure;
    plot(1:100,velocity_profile(:,1),'r');
    hold on
    plot(1:100,velocity_profile(:,2),'g');
    hold on
    plot(1:100,velocity_profile(:,3),'b');
    hold on
    plot(1:100,velocity_profile(:,4),'c');
    hold on
    plot(1:100,velocity_profile(:,5),'y');
    title('velocity of each joint for 1c');
    
        acc_profile = zeros(100,5);
    for i=2:100
        acc_profile(i,:) = (velocity_profile(i,:)-velocity_profile(i-1,:))./0.125;
    end
    figure;
    plot(1:100,acc_profile(:,1),'r');
    hold on
    plot(1:100,acc_profile(:,2),'g');
    hold on
    plot(1:100,acc_profile(:,3),'b');
    hold on
    plot(1:100,acc_profile(:,4),'c');
    hold on
    plot(1:100,acc_profile(:,5),'y');
    title('acceleration of each joint for 1c');
end

function [a,b,c] = trapezoidal_traj(qi,vi,ai,qf,vf,af,qv)
    a = [qi;zeros(1,5);qv/2];
    b = [qi-qv/2;qv;zeros(1,5)];
    c = [qf-(qv*3^2)/2; qv*3; -qv/2];
end