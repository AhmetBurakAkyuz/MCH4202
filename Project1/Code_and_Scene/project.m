% Reference: https://asl.ethz.ch/education/lectures/autonomous_mobile_robots/spring-2021.html

% Make sure to have the simulation scene example_scene.ttt running in V-REP!

% simulation setup, will add the matlab paths
connection = simulation_setup();

% the robot we want to interact with
robotNb = 0;

% open the connection
connection = simulation_openConnection(connection, robotNb);

% start simulation if not already started
simulation_start(connection);

% now enable stepped simulation mode:
simulation_setStepped(connection,true);

% and step 100 times:

[bodyDiameter, wheelDiameter, interWheelDist] = robot_init(connection);
% setting initial parameters, our robot is initially at rest :
set(gcf,'CurrentCharacter','@');
v = 0;
w = 0;
% reference: https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf
r = wheelDiameter / 2; % r = 0.195/2.0 (m);
l = interWheelDist; % l = 0.381 (m);
vel_left = 0;
vel_right = 0;
robot_setWheelSpeeds(connection, vel_left, vel_right);
fprintf('Simulation is starting...')

while true
    simulation_triggerStep(connection);
    

    % get wheel encoders
    [encoder_left, encoder_right] = robot_getEncoders(connection);

    % get wheel velocities
    [vel_left_wheel, vel_right_wheel] = robot_getWheelSpeeds(connection);

    % get laser data
    [laserDataX, laserDataY] = robot_getLaserData(connection);


    %% your code here %%
    k = get(gcf,'CurrentCharacter');
    if k == 'l'
        clc
        Ans = input('Please write the new linear velocity: ');
        v = Ans;
        [vel_left, vel_right] = Kinematic_Conv(v,w,r,l);
        robot_setWheelSpeeds(connection, vel_left, vel_right)
        Speeds = [v w];
        fprintf('The robots current linear speed is %.2f m/s and angular speed is %.2f rad/s', Speeds)
    elseif k == 'k'
        clc
        Ans = input('Please write the new angular velocity: ');
        w = Ans;
        [vel_left, vel_right] = Kinematic_Conv(v,w,r,l);
        robot_setWheelSpeeds(connection, vel_left, vel_right);
        Speeds = [v w];
        fprintf('The robots current linear speed is %.2f m/s and angular speed is %.2f rad/s', Speeds)

    elseif k == 's'
        clc
        Ans = 0;
        w = Ans;
        v = Ans;
        [vel_left, vel_right] = Kinematic_Conv(v,w,r,l);
        robot_setWheelSpeeds(connection, vel_left, vel_right);
        Speeds = [v w];
        if (~any(Speeds))
            fprintf('Robot has been stopped successfully!')
        end
    elseif k == 'z'
        clc
        fprintf('Simulation is terminating...')
        break;
    end
    set(gcf,'CurrentCharacter','@');
    k = get(gcf,'CurrentCharacter');
    


    pause(0.1);
end


% now disable stepped simulation mode:
simulation_setStepped(connection,false);

% stop the simulation
simulation_stop(connection);

% close the connection
simulation_closeConnection(connection);