% function [joint_profile, velocity_profile] = create_trajectory_1c(data)
%     [M, N] = size(data);
%     joint_profile = zeros(100, 5);
%     velocity_profile = zeros(100, 5);
%     constant_veloc = [0.15,0.3,0.15,0.3,0.3];
% 
%     position = zeros(5,3);
%     orientation = zeros(5,3);
%     zeroPoint = zeros(1,3);
%     for i = 1:5
%         position(i,1) = data(1,4,i);
%         position(i,2) = data(2,4,i);
%         position(i,3) = data(3,4,i);
%         orientation(i,:) = rotationMatrixToVector(data(1:3,1:3,i));
%     end
%     for i = 1:4
%         [xa(:,:,i) xb(:,:,i) xc(:,:,i)] = trapezoidal_traj(position(i,:),zeroPoint,zeroPoint,position(i+1,:),zeroPoint,zeroPoint,constant_veloc);
%         [ya(:,:,i) yb(:,:,i) yc(:,:,i)] = trapezoidal_traj(orientation(i,:),zeroPoint,zeroPoint,orientation(i+1,:),zeroPoint,zeroPoint,constant_veloc);
%     end
%     
%     k = 1;
%     for i = 1:4
%         for t = linspace(0,3,25)
%             if t <= 1
%                 output_position(k,:) = xa(1,:,i) + xa(2,:,i)*t + xa(3,:,i)*t^2;
%                 output_velocity(k,:) = xa(2,:,i) + 2*xa(3,:,i)*t;
%                 output_angle(k,:) = ya(1,:,i) + ya(2,:,i)*t + ya(3,:,i)*t^2;
%                 output_ang_velo(k,:) = ya(2,:,i) + 2*ya(3,:,i)*t;
%             elseif t <= 2
%                 output_position(k,:) = xb(1,:,i) + xb(2,:,i)*t + xb(3,:,i)*t^2;
%                 output_velocity(k,:) = xb(2,:,i) + 2*xb(3,:,i)*t;
%                 output_angle(k,:) = yb(1,:,i) + yb(2,:,i)*t + yb(3,:,i)*t^2;
%                 output_ang_velo(k,:) = yb(2,:,i) + 2*yb(3,:,i)*t;
%             else
%                 output_position(k,:) = xc(1,:,i) + xc(2,:,i)*t + xc(3,:,i)*t^2;
%                 output_velocity(k,:) = xc(2,:,i) + 2*xc(3,:,i)*t;
%                 output_angle(k,:) = yc(1,:,i) + yc(2,:,i)*t + yc(3,:,i)*t^2;
%                 output_ang_velo(k,:) = yc(2,:,i) + 2*yc(3,:,i)*t;
%             end
%             k = k + 1;
%         end
%     end
%     
%     matrix = zeros(4,4,100);
%     for i = 1:100
%         matrix(1,4,i) = output_position(i,1);
% 	    matrix(2,4,i) = output_position(i,2);
%         matrix(3,4,i) = output_position(i,3);
%         matrix(1:3,1:3,i) = rotationVectorToMatrix(output_angle(i,:)); 
%         matrix(4,4,i) = 1;
%         velo_matrix(1,4,i) = output_velocity(i,1);
% 	    velo_matrix(2,4,i) = output_velocity(i,2);
%         velo_matrix(3,4,i) = output_velocity(i,3);
%         velo_matrix(1:3,1:3,i) = rotationVectorToMatrix(output_ang_velo(i,:)); 
%         velo_matrix(4,4,i) = 1;
%     end
%     
%     L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
%     L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
%     L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
%     L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
%     L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
% 
%     %Construct the robot
%     planar5Rrobot = SerialLink([L1, L2, L3, L4, L5]);
%     for i=1:100
%         mask = [1,1,1,0,0,0];
%         if i==1
%             joint_profile(i,:) = planar5Rrobot.ikine(matrix(:,:,i), [0.1,0.1,0.1,0.1,0.1], mask, 'varstep');    
%         else
%             joint_profile(i,:) = planar5Rrobot.ikine(matrix(:,:,i),joint_profile(i-1,:) , mask, 'varstep');    
%         end
%     end
%     velocity_profile(1,:) = zeros(1,5);
%     for i=2:100
%         velocity_profile(i,:) = (joint_profile(i,:)-joint_profile(i-1,:))./0.125;
%     end
%     figure;
%     plot(1:100,velocity_profile(:,1),'r');
%     hold on
%     plot(1:100,velocity_profile(:,2),'g');
%     hold on
%     plot(1:100,velocity_profile(:,3),'b');
%     hold on
%     plot(1:100,velocity_profile(:,4),'c');
%     hold on
%     plot(1:100,velocity_profile(:,5),'y');
%     title('velocity of each joint for 1b');
% end
% 
% function [a,b,c] = trapezoidal_traj(qi,vi,ai,qf,vf,af,qv)
%     a = [qi;0;qv/2];
%     b = [qi-qv/2;qv;0];
%     c = [qf-(qv*3^2)/2; qv*3; -qv/2];
% end