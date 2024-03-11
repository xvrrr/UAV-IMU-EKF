function [state_predict, Fx, Fw] = predict(state, acc, omega, dt)
% Here are some values you need, all of them are [3*1] vectors
p = state(1:3);         % position
v = state(4:6);         % velocity
eu = state(7:9);        % Euler Angle
R = eul2mat(eu);        % Rotation Matrix
ba = state(10:12);      % bias of gyroscope
bg = state(13:15);      % bias of acceleration
g = [0, 0, -9.81]';     % gravity


state_predict = zeros(15, 1);
% The jacobian matrix that transfrom the angular velocity to the time
% derivitave of the euler angle which is realized by your self in Object 1
J = eul_jacobian(eu);
% ******************************************************************************************************
% PART 2: realize kinematic fuction and predict the state vector at the next time
% ****************************** WRITE YOR CODE HERE *************************************************

% calculate the predicted state with state transfrom function
% set sigma 





state_predict(1:3) = p+dt*v;
state_predict(4:6) = v+dt*(R*(acc-ba)+g);
state_predict(7:9) = eu+dt*(J*(omega-bg));
state_predict(10:12) = ba;
state_predict(13:15) = bg;




% *************************** WRITE YOUR CODE HERE *************************************************
% ****************************************************************************************************


Fx = eye(15);
% ******************************************************************************************************
% PART 3: calculate the first order derivitaves of state function
% ****************************** WRITE YOR CODE HERE *************************************************



% BE AWARE THAT F HAS ALREADY BEEN SET TO IDENTITY
% you can use code like

a = acc-ba;
b = omega-bg;


Fx(1:3,4:6) = eye(3) * dt;
Fx(7:9,10:12)=-J *dt;
Fx(4:6,13:15)=-R * dt;
Fx(4,7)=dt *( (-cos(theta) * sin(psi))*a(1) + (-cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi))*a(2)+(sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi))*a(3) );
Fx(4, 8) =dt *( (-sin(theta)*cos(psi))*a(1)+(sin(phi)*cos(theta)*cos(psi))*a(2)+(cos(phi)*cos(theta)*cos(psi))*a(3) );
Fx(4,9)=dt *( (sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi))*a(2)+(cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi))*a(3));
Fx(5, 7)=dt * ( (cos(theta)*cos(psi))*a(1)+(-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))*a(2)+(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi))*a(3) );
Fx(5,8)=dt *( (-sin(theta)*sin(psi))*a(1)+(sin(phi)*cos(theta)*sin(psi))*a(2)+(cos(phi)*cos(theta)*sin(psi))*a(3) );
Fx(5,9) = dt *( (-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi))*a(2)+(-cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi))*a(3) );
Fx(6,7)=0;
Fx(6,8)=dt *( (-cos(theta))*a(1)+(-sin(phi)*sin(theta))*a(2)+(-cos(phi)*sin(theta))*a(3) );
Fx(6,9)=dt *( (cos(phi)*cos(theta))*a(2)+(-sin(phi)*cos(theta))*a(3) );
Fx(7,7)=1 ;
Fx(7,8)=dt * ( ((-sin(phi)*sin(theta))/(-cos(theta)*cos(theta)))*b(2)+((-cos(phi)*sin(theta))/(-cos(theta)*cos(theta)))*b(3) );
Fx(7,9)=dt * ( (cos(phi)/cos(theta))*b(2)+(-sin(phi)/cos(theta))*b(3) );
Fx(8,7)=0 ;
Fx(8,8)=1 ;
Fx(8,9)=dt * ( (-sin(phi))*b(2)-cos(phi)*b(3) );
Fx(9,7)=0 ;
Fx(9,8)=dt * ( (sin(phi)/(cos(theta)*cos(theta)))*b(2)+(cos(phi)/(cos(theta)*cos(theta)))*b(3) );
Fx(9,9)=l+dt * ( cos(phi)*tan(theta)*b(2)-sin(phi)*tan(theta)*b(3) );

% to set the value of a block in F (row 1 to row 3, col 4 to col 6)
% or code like
%Fx(1, 1) = 1.0;
% to set the velue of an element in F (row 1, col 1)


% *************************** WRITE YOUR CODE HERE *************************************************
% ***************************************************************************************************


Fw = zeros(15, 12);
% ******************************************************************************************************
% PART 3: calculate the first order derivitaves of state function
% ****************************** WRITE YOR CODE HERE *************************************************

% BE AWARE THAT F HAS ALREADY BEEN SET TO IDENTITY
% you can use code like
Fw(4:6, 1:3)= -R*dt;
Fw(7:9, 4:6)= -J*dt;
Fw(13:15, 10:12) = eye(3)*dt;
Fw(10:12, 7:9)= eye(3)*dt;
% to set the value of a block in F (row 13 to row 15, col 10 to col 12)
% or code like
%Fw(1, 1) = 0.0;
% to set the velue of an element in F (row 1, col 1)


% *************************** WRITE YOUR CODE HERE *************************************************
% ***************************************************************************************************
end

