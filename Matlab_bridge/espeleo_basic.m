

%Definition of global velocities
global p q R
p = [0; 0; 0];
q = [1; 0; 0; 0];
R = eye(3);

a = 2; % ellipse x semi-axis
b = 1; % ellipse y semi-axis
Kp = 1; % controller proportional gain
d = 0.15; % distance d for the feedback linearization
vd = 0.5; % desired velocity
v_max = 1.5; % maximum velocity

% Initialize ROS comunication
rosshutdown
rosinit
% Define a subscriber to read the robot pose
sub_pose = rossubscriber('/tf',@callback_pose);
% Define a publisher to send velocity commands for the robot
[pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

% Time step for simulation
dt = 0.05; % inverse of publishing rate
% T = 100; 
% N = round(T/dt);

tau = 0; % parameter of the curve
d_tau = 0; % time derivative of parameter (used to impolse a constant velocity)

t = -dt;
% for k = 1:1:N
while (1)
    
    % Increment time
    t = t + dt;

    %Get Euler angles
    rpy = quat2eul(q');
    yaw = rpy(1);
    
    % Reference
    p_r = [a*cos(tau); b*sin(tau)];
    v_r = [-a*sin(tau)*d_tau; b*cos(tau)*d_tau];
    
    % Linear controller
    v = Kp*(p_r-p(1:2)) + v_r;
    
    if norm(v) > v_max
        v = v_max*v/norm(v);
    end
    
    % Feedback linearization
    vw = [cos(yaw) sin(yaw); -sin(yaw)/d cos(yaw)/d]*v;
    
    % Increment parameter (such that v_r = vd)
    d_tau = (vd/(norm([-a*sin(tau);b*cos(tau)])));
    tau = tau + d_tau*dt;
    
    % Publish velocity command
    twist_msg.Linear.X = vw(1);
    twist_msg.Angular.Z = vw(2);
    send(pub,twist_msg);
    
    % Pause
    pause(0.1)
    
end


















