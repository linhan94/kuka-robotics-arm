function [joint_profile, velocity_profile] = create_trajectory_1a(data)
    [M, N] = size(data);
    joint_profile = zeros(100, 5);
    velocity_profile = zeros(100, 5);
    zeroPoint = zeros(1,5);
    
    %Implement quintic polynomial
%     for i = 1:4
%         x(:,:,i) = quinticPoly(data(i,:),zeroPoint,zeroPoint,data(i+1,:),zeroPoint,zeroPoint,3*i-3,3*i);
%     end
%     
%     k = 1;
%     for t = linspace(0,3,25)
%         joint_profile(k,:) = x(1,:,1) + x(2,:,1)*t + x(3,:,1)*t^2 + x(4,:,1)*t^3 ...
%                              + x(5,:,1)*t^4 + x(6,:,1)*t^5;
%         velocity_profile(k,:) = x(2,:,1) + 2*x(3,:,1)*t + 3*x(4,:,1)*t^2 + ...
%                                 4*x(5,:,1)*t^3 + 5*x(6,:,1)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(3,6,25)
%         joint_profile(k,:) = x(1,:,2) + x(2,:,2)*t + x(3,:,2)*t^2 + x(4,:,2)*t^3 ...
%                              + x(5,:,2)*t^4 + x(6,:,2)*t^5;
%         velocity_profile(k,:) = x(2,:,2) + 2*x(3,:,2)*t + 3*x(4,:,2)*t^2 + ...
%                                 4*x(5,:,2)*t^3 + 5*x(6,:,2)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(6,9,25)
%         joint_profile(k,:) = x(1,:,3) + x(2,:,3)*t + x(3,:,3)*t^2 + x(4,:,3)*t^3 ...
%                              + x(5,:,3)*t^4 + x(6,:,3)*t^5;
%         velocity_profile(k,:) = x(2,:,3) + 2*x(3,:,3)*t + 3*x(4,:,3)*t^2 + ...
%                                 4*x(5,:,3)*t^3 + 5*x(6,:,3)*t^4;
%         k = k + 1;
%     end
%     for t = linspace(9,12,25)
%         joint_profile(k,:) = x(1,:,4) + x(2,:,4)*t + x(3,:,4)*t^2 + x(4,:,4)*t^3 ...
%                              + x(5,:,4)*t^4 + x(6,:,4)*t^5;
%         velocity_profile(k,:) = x(2,:,4) + 2*x(3,:,4)*t + 3*x(4,:,4)*t^2 + ...
%                                 4*x(5,:,4)*t^3 + 5*x(6,:,4)*t^4;
%         k = k + 1;
%     end
    % End of quintic polynomial

    % Implement cubic polynomial
    for i = 1:4
        x(:,:,i) = cubicPoly(data(i,:),zeroPoint,data(i+1,:),zeroPoint,3*i-3,3*i);
    end
    
    k = 1;
    t = linspace(0,3,25);
    for t = linspace(0,3,25)
        joint_profile(k,:) = x(1,:,1) + x(2,:,1)*t + x(3,:,1)*t^2 + x(4,:,1)*t^3;
        velocity_profile(k,:) = x(2,:,1) + 2*x(3,:,1)*t + 3*x(4,:,1)*t^2;
        k = k + 1;
    end
    for t = linspace(3,6,25)
        joint_profile(k,:) = x(1,:,2) + x(2,:,2)*t + x(3,:,2)*t^2 + x(4,:,2)*t^3;
        velocity_profile(k,:) = x(2,:,2) + 2*x(3,:,2)*t + 3*x(4,:,2)*t^2;
        k = k + 1;
    end
    for t = linspace(6,9,25)
        joint_profile(k,:) = x(1,:,3) + x(2,:,3)*t + x(3,:,3)*t^2 + x(4,:,3)*t^3;
        velocity_profile(k,:) = x(2,:,3) + 2*x(3,:,3)*t + 3*x(4,:,3)*t^2;
        k = k + 1;
    end
    for t = linspace(9,12,25)
        joint_profile(k,:) = x(1,:,4) + x(2,:,4)*t + x(3,:,4)*t^2 + x(4,:,4)*t^3;
        velocity_profile(k,:) = x(2,:,4) + 2*x(3,:,4)*t + 3*x(4,:,4)*t^2;
        k = k + 1;
    end
    
    acc_profile = zeros(100,5);
    for i=2:100
        acc_profile(i,:) = (velocity_profile(i,:)-velocity_profile(i-1,:))./0.125;
    end
    %end of cubic polynomial
    
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
    title('velocity of each joint for 1a');
    
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
    title('acceleration of each joint for 1a');
end

function x = quinticPoly(qi,vi,ai,qf,vf,af,ti,tf)
    A = [1, ti, ti^2, ti^3, ti^4, ti^5;
         0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4;
         0, 0, 2, 6*ti, 12*ti^2, 20*ti^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    b = [qi; vi; ai; qf; vf; af];
    x = A\b;
end

function x = cubicPoly(qi,vi,qf,vf,ti,tf)
    A = [1, ti, ti^2, ti^3;
         0, 1, 2*ti, 3*ti^2;
         1, tf, tf^2, tf^3;
         0, 1, 2*tf, 3*tf^2];
    b = [qi;vi;qf;vf];
    x = A\b;
end