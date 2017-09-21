function [joint_profile, velocity_profile] = create_trajectory_1b(data)
    [M, N] = size(data);
    joint_profile = zeros(100, 5);
    velocity_profile = zeros(100, 5);
    position = zeros(5,3);
    orientation = zeros(5,4);
    zeroPoint = zeros(1,3);
    for i = 1:5
        position(i,1) = data(1,4,i);
        position(i,2) = data(2,4,i);
        position(i,3) = data(3,4,i);
        orientation(i,:) = rotm2quat(data(1:3,1:3,i));
    end
    for i = 1:4
        % To do quintic interpolation, uncomment line 16 and 22 to 50, comment line 
        %x(:,:,i) = quinticPoly(position(i,:),zeroPoint,zeroPoint,position(i+1,:),zeroPoint,zeroPoint,3*i-3,3*i);
        % To do linear interpolation, comment line 16 and 22 to 50, uncomment 18
        output_position((i-1)*25+1:i*25,:) = linearIn(position(i,:),position(i+1,:),25);
        output_angle((i-1)*25+1:i*25,:) = slerp(orientation(i,:),orientation(i+1,:),25);
    end

%     k = 1;
%     for t = linspace(0,3,25)
%         output_position(k,:) = x(1,:,1) + x(2,:,1)*t + x(3,:,1)*t^2 + x(4,:,1)*t^3 ...
%                              + x(5,:,1)*t^4 + x(6,:,1)*t^5;
%         output_velocity(k,:) = x(2,:,1) + 2*x(3,:,1)*t + 3*x(4,:,1)*t^2 + ...
%                                 4*x(5,:,1)*t^3 + 5*x(6,:,1)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(3,6,25)
%         output_position(k,:) = x(1,:,2) + x(2,:,2)*t + x(3,:,2)*t^2 + x(4,:,2)*t^3 ...
%                              + x(5,:,2)*t^4 + x(6,:,2)*t^5;
%         output_velocity(k,:) = x(2,:,2) + 2*x(3,:,2)*t + 3*x(4,:,2)*t^2 + ...
%                                 4*x(5,:,2)*t^3 + 5*x(6,:,2)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(6,9,25)
%         output_position(k,:) = x(1,:,3) + x(2,:,3)*t + x(3,:,3)*t^2 + x(4,:,3)*t^3 ...
%                              + x(5,:,3)*t^4 + x(6,:,3)*t^5;
%         output_velocity(k,:) = x(2,:,3) + 2*x(3,:,3)*t + 3*x(4,:,3)*t^2 + ...
%                                 4*x(5,:,3)*t^3 + 5*x(6,:,3)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(9,12,25)
%         output_position(k,:) = x(1,:,4) + x(2,:,4)*t + x(3,:,4)*t^2 + x(4,:,4)*t^3 ...
%                              + x(5,:,4)*t^4 + x(6,:,4)*t^5;
%         output_velocity(k,:) = x(2,:,4) + 2*x(3,:,4)*t + 3*x(4,:,4)*t^2 + ...
%                                 4*x(5,:,4)*t^3 + 5*x(6,:,4)*t^4;
%         k = k + 1;
%     end
    
    matrix = zeros(4,4,100);
    for i = 1:100
        matrix(1,4,i) = output_position(i,1);
	    matrix(2,4,i) = output_position(i,2);
        matrix(3,4,i) = output_position(i,3);
        matrix(1:3,1:3,i) = quat2rotm(output_angle(i,:)); 
        matrix(4,4,i) = 1;
    end

    for i=1:100
        joint_profile(i,:) = ikine_closedform(matrix(:,:,i));
    end
    
    velocity_profile(1,:) = zeros(1,5);
    for i=2:100
        velocity_profile(i,:) = (joint_profile(i,:)-joint_profile(i-1,:))./0.125;
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
    title('velocity of each joint for 1b');
    
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
    title('acceleration of each joint for 1b');
end

% qunintic polynomial interpolation
function xx = quinticPoly(qi,vi,ai,qf,vf,af,ti,tf)
    A = [1, ti, ti^2, ti^3, ti^4, ti^5;
         0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4;
         0, 0, 2, 6*ti, 12*ti^2, 20*ti^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    b = [qi; vi; ai; qf; vf; af];
    xx = A\b;
end

% spherical linear interpolation
function qm = slerp(qi, qf, split_num)
    if dot(qi,qf) >= 0
        theta = acos(dot(qi,qf));
    else
        qf = -qf;
        theta = acos(-dot(qi,qf));
    end
    
    qm = zeros(split_num,4);
    for i=1:split_num
        qm(i,:) = (qi*sin((1-(i-1)/split_num)*theta)+qf*sin(theta*(i-1)/split_num))/sin(theta);
    end
end

%linear interpolation
function pm = linearIn(pi,pf,split_num)
    pm = zeros(split_num,3);
    for i=0:split_num-1
        pm(i+1,:) = pi + i/split_num*(pf-pi);
    end
end