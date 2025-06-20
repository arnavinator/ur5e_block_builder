%% initialize IMU and calibrate
clear 
%% 
clear a; clear imu;

a = arduino()

imu = bno055(a)

status = readCalibrationStatus(imu)
%% check time between consecutive read cycles and determine Z_FORW
clear tic;
clear toc;
tic;
imu_read = read(imu);
imu_matrix = imu_read{:,:};
imu_mean = mean(imu_matrix);

Z_FORW = imu_mean(10)

%% setup ros commands
% setup armCmd
armCmd = rospublisher('/scaled_pos_joint_traj_controller/command');

% Set up a publisher to send the gripper commands:
gripperpub = rospublisher('/gripper_service_es159','std_msgs/Float32MultiArray');
gripperMsg = rosmessage('std_msgs/Float32MultiArray');

% setup camera subscriber
CamSub = rossubscriber('/usb_cam/image_raw');
CamMsg = receive(CamSub);

% setup end-effector state subscriber
possub = rossubscriber('tf');


%% initialize grabbing data

% Create a message that corresponds to moving the arm out of the way of the camera field of view:
OriginalMsg = rosmessage(armCmd);
OriginalMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
originalpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
originalpoint.Positions = [0 -pi/2 0 0 0 0];
originalpoint.Velocities = [0 0 0 0 0 0];
originalpoint.TimeFromStart.Sec = 5;
OriginalMsg.Points = [originalpoint];

% take snapshot and process... only need do once!
send(armCmd, OriginalMsg);
while ismoving()
    % ismoving() returns true while arm is moving... do nothing if moving
end
CamMsg = receive(CamSub);
reshaped_data = reshape(CamMsg.Data, [3, (length(CamMsg.Data)/3)]).';

% width is # of rows, height is # of columns, but need to transpose
% since need to reshape elements row-first not column-first
Red = reshape(reshaped_data(:,1), [CamMsg.Width, CamMsg.Height]).';
Green = reshape(reshaped_data(:,2), [CamMsg.Width, CamMsg.Height]).';
Blue = reshape(reshaped_data(:,3), [CamMsg.Width, CamMsg.Height]).';

% Crop the images (crop y-dim a little extra to remove sign)
Red = Red(151:400,51:550);
Green = Green(151:400,51:550);
Blue = Blue(151:400,51:550);

% do thresholding based on color channels to get all blocks
RedOnly = ((Red > 150) + (Green < 120) + (Blue < 120));
RedOnly = conv2(RedOnly/3, ones(4,4), "same") > 13;

M1 = Red > 220;
M2 = Red < 100;
M3 = M1 + M2;
M3 = conv2(M3/3, ones(4,4), "same");
M3 = M3 > 4;

M3 = (RedOnly+M3) > 0.5;

% detect blobs and find their centriods
box = regionprops(M3, 'Centroid'); 
centroids = cat(1, box.Centroid);

% display valid blocks detected and actual image taken
figure;
imshow(double(cat(3, Red, Green, Blue))/255);
h = gca;
h.Visible = 'On';


% find top left red centroid, and remove from valid list of blocks
[min_val, id_min] = min(centroids(:,1));
topcentroid_pixel = centroids(id_min,:);
centroids(id_min,:) = [];

% find bottom right red centroid, and remove from valid list of blocks
[max_val, id_max] = max(centroids(:,1));
bottomcentroid_pixel = centroids(id_max,:);
centroids(id_max,:) = [];

% display centroids minus the reference squares
figure;
hold on;
imagesc(M3); colormap(gray);
plot(centroids(:,1),centroids(:,2),'b*')
set(gca, 'YDir','reverse');
hold off

% figure out scaling given real world red reference corner positions
% given from lab5 top corner and bottom corner, and that the y-dist
% between corners is 825 == 33 grid blocks, given each reference square
% has length 3, we offset all corners by 1.5 grid blocks in order to
% convert to centroids in real world reference
topcentroid = [340+37.5,-400+37.5];
bottomcentroid = [715-37.5,425-37.5];

% change in real dist / change in pixel dist
xScale = (bottomcentroid(1) - topcentroid(1)) / (bottomcentroid_pixel(2) - topcentroid_pixel(2));
yScale = (bottomcentroid(2) - topcentroid(2)) / (bottomcentroid_pixel(1) - topcentroid_pixel(1));
%%
% move gripper to initial orientation with (x = 650, y = 0, z = 100)mm
Tinitial = [-1 0 0 665; 0 1 0 0; 0 0 -1 100; 0 0 0 1]; 
guessinitial = [-10 -60 90 -120 -90 80]*(pi/180);
initialTheta = IKshell(Tinitial, guessinitial);
InitialPosMsg = rosmessage(armCmd);
InitialPosMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
initialpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
initialpoint.Positions = initialTheta;
initialpoint.Velocities = [0 0 0 0 0 0];
initialpoint.TimeFromStart.Sec = 5;
InitialPosMsg.Points = [initialpoint];
send(armCmd, InitialPosMsg);
while ismoving()
  % ismoving() returns true while arm is moving... do nothing if moving
end

%% past position tracking
x_prev = 665;
y_prev = 0;
z_prev = 100;

%% block grab tracking
block_grabbed = false;      % keep track of whether gripper open or closed 

%% interface for block manipulation (auto grab + gripper control + IMU control) 
closedgripper = 255;        % Value to send the gripper if you want it to close fully
opengripper = 0;            % Value to send the gripper if you want it to open fully
blockgripper = 147;         % Value to send the gripper if you want it to close on the block
gripperspeed = 55;          % Use this value for speed
gripperforce = 55;          % Use this value for force
pause(5);
cont = true;
count = 0;
while cont    % constant loop checking inputs
    count = count + 1;

    if readVoltage(a, 'A7') > 3.0    % auto grab requested by user interface 
        disp("AUTO-GRAB")
        if isempty(centroids)
            error("All blocks have been used!");
        elseif block_grabbed == true
            error("Block is already grabbed!")
        else
            block_grabbed = true;
        end
        
        % determine block to grab (assuming this action is allowed)
        [min_val, id_min] = min(centroids(:,1));
        blockPos_pixel = centroids(id_min,:);
        centroids(id_min,:) = [];  % remove block from valid grabbing list
        blockPos = [(blockPos_pixel(2)-topcentroid_pixel(2))*xScale + topcentroid(1)...
                (blockPos_pixel(1)-topcentroid_pixel(1))*yScale + topcentroid(2) ];
        
        % move end-effector to z = 100m while maintaining (x,y)
        startPos = [y_prev, z_prev];
        endPos = [y_prev, 100];
        steps = 5;
        step_size = edist(startPos, endPos)/steps;
        time_per_pose = 1;
        WaypointOutput = [linspace(startPos(1), endPos(1), steps).'  ...
                          linspace(startPos(2), endPos(2), steps).'];
        % calculate joint angle for desired trajectory given waypoint, initial 
        % theta guess, and fixed X position
        thetas = Z_executeTrajectory(WaypointOutput, initialTheta, x_prev);
        clear commandlist; clear shapeMsg;
        N = length(thetas(:,1));
        for i=1:N
            commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            commandlist(i).Positions = thetas(i,:);
            commandlist(i).TimeFromStart.Sec = i*time_per_pose;
            % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
            if i<N
                commandlist(i).Velocities = (thetas(i+1,:) - thetas(i,:))/time_per_pose;
            else
                commandlist(i).Velocities = zeros(6,1);
            end
        end
        shapeMsg = rosmessage(armCmd);
        shapeMsg.Points = [commandlist];
        shapeMsg.JointNames = {'shoulder_pan_joint';
        'shoulder_lift_joint'; 'elbow_joint'; 'wrist_1_joint'; 
        'wrist_2_joint'; 'wrist_3_joint'};
        send(armCmd,shapeMsg);
        while ismoving()
          % ismoving() returns true while arm is moving... do nothing if moving
        end
        
    
        % move end-effector's (x,y) to blockPos
        startPos = [x_prev, y_prev];
        endPos = blockPos;
        steps = 10;
        step_size = edist(startPos, endPos)/steps;
        time_per_pose = 1;
        WaypointOutput = [linspace(startPos(1), endPos(1), steps).'  ...
                          linspace(startPos(2), endPos(2), steps).'];
        % calculate joint angle for desired trajectory given waypoint, initial 
        % theta guess, and fixed Z position
        thetas = executeTrajectory(WaypointOutput, thetas(end,:), 100);
        clear commandlist; clear shapeMsg;
        N = length(thetas(:,1));
        for i=1:N
            commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            commandlist(i).Positions = thetas(i,:);
            commandlist(i).TimeFromStart.Sec = i*time_per_pose;
            % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
            if i<N
                commandlist(i).Velocities = (thetas(i+1,:) - thetas(i,:))/time_per_pose;
            else
                commandlist(i).Velocities = zeros(6,1);
            end
        end
        shapeMsg = rosmessage(armCmd);
        shapeMsg.Points = [commandlist];
        shapeMsg.JointNames = {'shoulder_pan_joint';
        'shoulder_lift_joint'; 'elbow_joint'; 'wrist_1_joint'; 
        'wrist_2_joint'; 'wrist_3_joint'};
        send(armCmd,shapeMsg);
        while ismoving()
          % ismoving() returns true while arm is moving... do nothing if moving
        end        
    
        % open the gripper
        gripperMsg.Data = [opengripper gripperspeed gripperforce];
        send(gripperpub,gripperMsg);
        pause(5) % wait for gripper to finish moving... ismoving() only for arm, not gripper
    
        % lower the end-effector to z = -150mm
        startPos = [blockPos(2), 100];
        endPos = [blockPos(2), -150];
        steps = 10;
        step_size = edist(startPos, endPos)/steps;
        time_per_pose = 1;
        WaypointOutput = [linspace(startPos(1), endPos(1), steps).'  ...
                          linspace(startPos(2), endPos(2), steps).'];
        % calculate joint angle for desired trajectory given waypoint, initial 
        % theta guess, and fixed X position
        thetas = Z_executeTrajectory(WaypointOutput, thetas(end,:), blockPos(1));
        clear commandlist; clear shapeMsg;
        N = length(thetas(:,1));
        for i=1:N
            commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            commandlist(i).Positions = thetas(i,:);
            commandlist(i).TimeFromStart.Sec = time_per_pose*i;
            % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
            if i<N
                commandlist(i).Velocities = (thetas(i+1,:) - thetas(i,:))/time_per_pose;
            else
                commandlist(i).Velocities = zeros(6,1);
            end
        end
        shapeMsg = rosmessage(armCmd);
        shapeMsg.Points = [commandlist];
        shapeMsg.JointNames = {'shoulder_pan_joint';
        'shoulder_lift_joint'; 'elbow_joint'; 'wrist_1_joint'; 
        'wrist_2_joint'; 'wrist_3_joint'};
        send(armCmd,shapeMsg);
        while ismoving()
          % ismoving() returns true while arm is moving... do nothing if moving
        end
        
    
        % close the gripper on the block
        gripperMsg.Data = [blockgripper gripperspeed gripperforce];
        send(gripperpub,gripperMsg);
        pause(5) % wait for gripper to finish moving
        
        % Raise the block to z = 50mm, while still maintaining (x,y) 
        startPos = [blockPos(2), -130];
        endPos = [blockPos(2), 50];
        steps = 10;
        step_size = edist(startPos, endPos)/steps;
%         time_per_pose = (step_size*0.5/100)/0.2;
        time_per_pose = 1;
        WaypointOutput = [linspace(startPos(1), endPos(1), steps).'  ...
                          linspace(startPos(2), endPos(2), steps).'];
        % calculate joint angle for desired trajectory given waypoint, initial 
        % theta guess, and fixed X position
        thetas = Z_executeTrajectory(WaypointOutput, thetas(end,:), blockPos(1));
        clear commandlist; clear shapeMsg;
        N = length(thetas(:,1));
        for i=1:N
            commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            commandlist(i).Positions = thetas(i,:);
            commandlist(i).TimeFromStart.Sec = time_per_pose*i;
            % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
            if i<N
                commandlist(i).Velocities = (thetas(i+1,:) - thetas(i,:))/time_per_pose;
            else
                commandlist(i).Velocities = zeros(6,1);
            end
        end
        shapeMsg = rosmessage(armCmd);
        shapeMsg.Points = [commandlist];
        shapeMsg.JointNames = {'shoulder_pan_joint';
        'shoulder_lift_joint'; 'elbow_joint'; 'wrist_1_joint'; 
        'wrist_2_joint'; 'wrist_3_joint'};
        send(armCmd,shapeMsg);
        while ismoving()
          % ismoving() returns true while arm is moving... do nothing if moving
        end
        
        x_prev = blockPos(1);
        y_prev = blockPos(2);
        z_prev = 50;
    
    elseif readVoltage(a, 'A6') > 3.0
        disp("BLOCK-RELEASE")
        % open the gripper to drop a block!
        gripperMsg.Data = [opengripper gripperspeed gripperforce];
        send(gripperpub,gripperMsg);
        block_grabbed = false;
        pause(5) % wait for gripper to finish moving... ismoving() only for arm, not gripper
    
    else    % otherwise read inputs from IMU interface to control end-effector
        disp("IMU-PILOT")
        % minimize pin reads so buffer fills up slower
        % POT-toggled displacment_scale 
        % (readVoltage(a, 'A0')/3.3 + 60) has range 0 to 60
        % min speed of 10mm/s, max of 70mm/s in x/y/z per measurement
        displacement_scale = 10 + readVoltage(a, 'A0')/3.3*60;  
        % fix displacement_scale for 5 reads of 1s movements
        for i=1:5
            imu_read = read(imu);
            imu_matrix = imu_read{:,:};
            imu_mean = mean(imu_matrix);

            % Read Orientation values (Euler angles) from the sensor
            z_orient = imu_mean(10);  % yaw --> z
            y_orient = imu_mean(11);  % pitch --> y
            x_orient = imu_mean(12);  % roll --> x

            % in the motion to which the remote is ergonomically constrained
            % to, we expect x,y \in [-pi/2, pi/2], z \in [0, 2pi]
            % sometimes when moving x, y=0 will jump to +/- pi... map it back to 0! 
            if (y_orient > 3*pi/4 || y_orient < -3*pi/4)
                y_orient = 0;
            end
            % in case of the current orientation, Z is latched onto the current
            % orientation of 2*pi but quickly can switch to 0... so want it to
            % be continuous transition when evaluating relative position
            % if Z_FORW near 0, then if z_orient near 2pi, subtract 2pi
            if (Z_FORW >= 0 && Z_FORW <= pi/2 && z_orient >= 3*pi/2)
                z_orient = z_orient - 2*pi;
            % if Z_FORW near 2pi, then if z_orient near 0, add 2pi
            elseif (Z_FORW >= 3*pi/2 && Z_FORW <= 2*pi && z_orient >= 3*pi/2)
                z_orient = z_orient - 2*pi;
            end

    %         % read currrent position of end-effector
    %         % this is given in meters... need to convert to mm
    %         clear posmsg;
    %         posmsg = receive(possub);  
    %         x1 = posmsg.Transforms(1).Transform.Translation.X*-1000;
    %         y1 = posmsg.Transforms(1).Transform.Translation.Y*1000;
    %         z1 = posmsg.Transforms(1).Transform.Translation.Z*1000;

            % only accept value to displace if above a slight noise threshold
            % of 0.05*pi
            % if Roll angle is decreasing, map it to increasing x
            if (abs(x_orient) > 0.05*pi)
                delta_x = -1*x_orient*displacement_scale;
            else
                delta_x = 0;
            end
            % if Pitch angle is decreasing, map it to decreasing y
            if (abs(y_orient) > 0.05*pi)
                delta_y = -1*y_orient*displacement_scale;   
            else
                delta_y = 0;
            end
            % we only consider z_displacement if x and y have no displacement
            % if Yaw angle is increasing (given no Roll/Pitch), map it to
            % decreasing z
            % Z_FORW is the Yaw angle corresponding to facing directly at
            % robot, ie this is rest position we interpret as no z-change of
            % controller
            % due to drift on 
            if (abs(x_orient) < 0.05*pi && abs(y_orient) < 0.05*pi && abs(z_orient-Z_FORW) > 0.1*pi)
                delta_z = -1*(z_orient-Z_FORW)*displacement_scale;
            else
                delta_z = 0;
            end

%             disp("Pre  x: " + x_prev + " y: " + y_prev + " z: " + z_prev);
            disp("delta x: " + (delta_x) + " delta y: " + (delta_y) + " delta z: " + (delta_z));

            x_prev = delta_x + x_prev;
            y_prev = delta_y + y_prev;
            z_prev = delta_z + z_prev;
            
            % keep from crashing into the table
            if z_prev < -150
                z_prev = -150;
            end
            T2 = [-1 0 0 (x_prev); 
                        0 1 0 (y_prev); 
                        0 0 -1 (z_prev); 
                        0 0 0 1];


            % if thetas exist, just trim it so that doesn't keep growing, else
            % pick initial guess for IK algorithm
            if exist('thetas', 'var')
                thetas = thetas(end, :);
            else
                thetas = [-10 -60 90 -120 -90 80]*(pi/180);
            end
            clear InitialPosMsg; clear initialpoint;
            initialTheta = IKshell(T2, thetas(end,:));
            InitialPosMsg = rosmessage(armCmd);
            InitialPosMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
            initialpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            initialpoint.Positions = initialTheta;
            initialpoint.Velocities = [0 0 0 0 0 0];
            initialpoint.TimeFromStart.Sec = 1;
            InitialPosMsg.Points = [initialpoint];
            send(armCmd, InitialPosMsg);

            while (ismoving())
              % ismoving() returns true while arm is moving
            end

    %         clear posmsg;
    %         posmsg = receive(possub);  
    %         x1 = posmsg.Transforms(1).Transform.Translation.X*-1000;
    %         y1 = posmsg.Transforms(1).Transform.Translation.Y*1000;
    %         z1 = posmsg.Transforms(1).Transform.Translation.Z*1000;
    %         disp("Post x: " + x1 + " y: " + y1 + " z: " + z1);
    % 
    %         
%             disp("Post x: " + x_prev + " y: " + y_prev + " z: " + z_prev);
        end
    end

end


%% function definitions
% Returns true if the end effector has a nonzero velocity
function ismoving = ismoving()
    possub = rossubscriber('tf');
    
    posmsg1 = receive(possub);
  
    x1 = posmsg1.Transforms(1).Transform.Translation.X;
    y1 = posmsg1.Transforms(1).Transform.Translation.Y;
    z1 = posmsg1.Transforms(1).Transform.Translation.Z;
    rx1 = posmsg1.Transforms(1).Transform.Rotation.X;
    ry1 = posmsg1.Transforms(1).Transform.Rotation.Y;
    rz1 = posmsg1.Transforms(1).Transform.Rotation.Z;
    
    pause(.02);
    
    posmsg2 = receive(possub);
    x2 = posmsg2.Transforms(1).Transform.Translation.X;
    y2 = posmsg2.Transforms(1).Transform.Translation.Y;
    z2 = posmsg2.Transforms(1).Transform.Translation.Z;
    rx2 = posmsg2.Transforms(1).Transform.Rotation.X;
    ry2 = posmsg2.Transforms(1).Transform.Rotation.Y;
    rz2 = posmsg2.Transforms(1).Transform.Rotation.Z;

    deltapos = [abs(x2-x1),abs(y2-y1),abs(z2-z1),abs(rx2-rx1),abs(ry2-ry1),abs(rz2-rz1)];
    if sum(deltapos) >= 1e-4 % Make stricter if necessary
      ismoving = true;
    else
      ismoving = false;
    end
end


function theta = IKshell(T,theta0)
    % Numerical calculation of UR5e inverse kinematics for end-effector position described by the transformation matrix T, starting from initial guess theta0 for the angles.
    
    % First just make sure theta0 is a column vector, and exit if not
    if size(theta0,1)==1, theta0 = theta0'; elseif size(theta0,2)~=1, disp('Initial guess needs to be a 1D vector'); return, end
    
    % Repeating the arm geometry from the FK lab; all values in mm:
    W2 = 259.6;
    W1 = 133.3;
    H2 = 99.7;
    H1 = 162.5;
    L1 = 425;
    L2 = 392.2;
    
    % Screw axes in the fixed (world) frame:
    w1 = [0 0 1];
    w2 = [0 1 0];
    w3 = [0 1 0];
    w4 = [0 1 0];
    w5 = [0 0 -1];
    w6 = [0 1 0];
    q1 = [0 0 0];
    q2 = [0 0 H1];
    q3 = [L1 0 H1];
    q4 = [L1+L2 0 H1];
    q5 = [L1+L2 W1 0];
    q6 = [L1+L2 0 H1-H2];
    v1 = cross(-w1,q1);
    v2 = cross(-w2,q2);
    v3 = cross(-w3,q3);
    v4 = cross(-w4,q4);
    v5 = cross(-w5,q5);
    v6 = cross(-w6,q6);
    S1 = [w1 v1];
    S2 = [w2 v2];
    S3 = [w3 v3];
    S4 = [w4 v4];
    S5 = [w5 v5];
    S6 = [w6 v6];
    M = [-1 0 0 (L1+L2); 0 0 1 (W1+W2); 0 1 0 H1-H2; 0 0 0 1];
    % Transformation between the world and body frames (when at home position) -- M above and T below should be (and are) the same:
    Rab = [ [-1 0 0]' [0 0 1]' [0 1 0]' ];
    Tab = [Rab [(L1+L2) (W1+W2) H1-H2]'; 0 0 0 1];
    % Bracketed versions, S1 -> [S1] etc (definition at Eqn 3.85 in the book):
    S1b = [0 -S1(3) S1(2) S1(4) ; S1(3) 0 -S1(1) S1(5) ; -S1(2) S1(1) 0 S1(6) ; 0 0 0 0];
    S2b = [0 -S2(3) S2(2) S2(4) ; S2(3) 0 -S2(1) S2(5) ; -S2(2) S2(1) 0 S2(6) ; 0 0 0 0];
    S3b = [0 -S3(3) S3(2) S3(4) ; S3(3) 0 -S3(1) S3(5) ; -S3(2) S3(1) 0 S3(6) ; 0 0 0 0];
    S4b = [0 -S4(3) S4(2) S4(4) ; S4(3) 0 -S4(1) S4(5) ; -S4(2) S4(1) 0 S4(6) ; 0 0 0 0];
    S5b = [0 -S5(3) S5(2) S5(4) ; S5(3) 0 -S5(1) S5(5) ; -S5(2) S5(1) 0 S5(6) ; 0 0 0 0];
    S6b = [0 -S6(3) S6(2) S6(4) ; S6(3) 0 -S6(1) S6(5) ; -S6(2) S6(1) 0 S6(6) ; 0 0 0 0];
    % Converting from world frame to body frame (Eqn 3.75, and just below Eqn 4.16):
    B1b = inv(Tab)*S1b*Tab;
    B2b = inv(Tab)*S2b*Tab;
    B3b = inv(Tab)*S3b*Tab;
    B4b = inv(Tab)*S4b*Tab;
    B5b = inv(Tab)*S5b*Tab;
    B6b = inv(Tab)*S6b*Tab;
    % Write these in the non-bracketed versions as well, [B1] -> B1 etc (3.85 again):
    B1 = [B1b(3,2) B1b(1,3) B1b(2,1) B1b(1,4) B1b(2,4) B1b(3,4)]';
    B2 = [B2b(3,2) B2b(1,3) B2b(2,1) B2b(1,4) B2b(2,4) B2b(3,4)]';
    B3 = [B3b(3,2) B3b(1,3) B3b(2,1) B3b(1,4) B3b(2,4) B3b(3,4)]';
    B4 = [B4b(3,2) B4b(1,3) B4b(2,1) B4b(1,4) B4b(2,4) B4b(3,4)]';
    B5 = [B5b(3,2) B5b(1,3) B5b(2,1) B5b(1,4) B5b(2,4) B5b(3,4)]';
    B6 = [B6b(3,2) B6b(1,3) B6b(2,1) B6b(1,4) B6b(2,4) B6b(3,4)]';
    % The upper-left 3x3 of each bracketed screw axis is the bracketed omega-hat encoding angle (3.85):
    w1b = [B1b(1:3,1:3)];
    w2b = [B2b(1:3,1:3)];
    w3b = [B3b(1:3,1:3)];
    w4b = [B4b(1:3,1:3)];
    w5b = [B5b(1:3,1:3)];
    w6b = [B6b(1:3,1:3)];
    
    % From here on follows the iterative algorithm described above Example 6.1, starting "To modify this algorithm to work with a desired end-effector configuration represented as T_sd...":
    
    thguess = theta0;  % initialize the current guess to the user-supplied value
    lastguess = thguess * 10 + 50;  % arbitrary value far away from the initial guess, so the while loop is entered
    
    while norm(thguess-lastguess) > 1e-3  % this isn't exactly the termination condition the book uses, but loop termination isn't the problem here
      lastguess = thguess;
      % split up the current guess into individual thetas for conciseness below
      t1 = thguess(1); t2 = thguess(2); t3 = thguess(3); t4 = thguess(4); t5 = thguess(5); t6 = thguess(6);
    
      % Exponential coordinate representation of rigid-body motions (take the screw axes above and write them in (R,d) form -- eB1 means exp([B1]theta1), etc.) (3.51, 3.86, 3.87):
      eB1 = [ [eye(3) + sin(t1)*w1b + (1-cos(t1))*(w1b*w1b)] (eye(3)*t1 + (1-cos(t1))*w1b + (t1-sin(t1))*(w1b*w1b))*B1(4:6) ; 0 0 0 1];
      eB2 = [ [eye(3) + sin(t2)*w2b + (1-cos(t2))*(w2b*w2b)] (eye(3)*t2 + (1-cos(t2))*w2b + (t2-sin(t2))*(w2b*w2b))*B2(4:6) ; 0 0 0 1];
      eB3 = [ [eye(3) + sin(t3)*w3b + (1-cos(t3))*(w3b*w3b)] (eye(3)*t3 + (1-cos(t3))*w3b + (t3-sin(t3))*(w3b*w3b))*B3(4:6) ; 0 0 0 1];
      eB4 = [ [eye(3) + sin(t4)*w4b + (1-cos(t4))*(w4b*w4b)] (eye(3)*t4 + (1-cos(t4))*w4b + (t4-sin(t4))*(w4b*w4b))*B4(4:6) ; 0 0 0 1];
      eB5 = [ [eye(3) + sin(t5)*w5b + (1-cos(t5))*(w5b*w5b)] (eye(3)*t5 + (1-cos(t5))*w5b + (t5-sin(t5))*(w5b*w5b))*B5(4:6) ; 0 0 0 1];
      eB6 = [ [eye(3) + sin(t6)*w6b + (1-cos(t6))*(w6b*w6b)] (eye(3)*t6 + (1-cos(t6))*w6b + (t6-sin(t6))*(w6b*w6b))*B6(4:6) ; 0 0 0 1];
    
      % To calculate the Jacobian, we need each of the body screw axes to be transformed by all the joints closer to the root, which is easiest to do using the bracketed form of the screw axes (5.13 and the line above):
      bJ1 = inv(eB6)*inv(eB5)*inv(eB4)*inv(eB3)*inv(eB2)*B1b*eB2*eB3*eB4*eB5*eB6;
      bJ2 = inv(eB6)*inv(eB5)*inv(eB4)*inv(eB3)*B2b*eB3*eB4*eB5*eB6;
      bJ3 = inv(eB6)*inv(eB5)*inv(eB4)*B3b*eB4*eB5*eB6;
      bJ4 = inv(eB6)*inv(eB5)*B4b*eB5*eB6;
      bJ5 = inv(eB6)*B5b*eB6;
      bJ6 = B6b;
      % The Jacobian is the set of column vectors which are the non-bracketed forms of the above (3.85, 5.13, 5.14):
      J1 = [bJ1(3,2) bJ1(1,3) bJ1(2,1) bJ1(1,4) bJ1(2,4) bJ1(3,4)]';
      J2 = [bJ2(3,2) bJ2(1,3) bJ2(2,1) bJ2(1,4) bJ2(2,4) bJ2(3,4)]';
      J3 = [bJ3(3,2) bJ3(1,3) bJ3(2,1) bJ3(1,4) bJ3(2,4) bJ3(3,4)]';
      J4 = [bJ4(3,2) bJ4(1,3) bJ4(2,1) bJ4(1,4) bJ4(2,4) bJ4(3,4)]';
      J5 = [bJ5(3,2) bJ5(1,3) bJ5(2,1) bJ5(1,4) bJ5(2,4) bJ5(3,4)]';
      J6 = [bJ6(3,2) bJ6(1,3) bJ6(2,1) bJ6(1,4) bJ6(2,4) bJ6(3,4)]';
      J = [J1 J2 J3 J4 J5 J6];
    
      % Forward kinematics for the robot's position as a function of the thetas (4.16):
      Tab = M*eB1*eB2*eB3*eB4*eB5*eB6;
    
      % T_bd = T^-1_ab * T_ad (substituting the current guesses for the thetas into the forward kinematics function):
      Tbd = inv(Tab)*T;
    
      % calculate theta and [w] for the matrix logarithm of T_bd (step (b) in the algorithm of section 3.3.3.2, Eqn 3.61):
      if norm(Tbd(1:3,1:3)-eye(3))<1e-5  % case 1: rotation is the identity matrix
         wbd = zeros(3); vbd = Tbd(1:3,4)/norm(Tbd(1:3,4)); thbd = norm(Tbd(1:3,4));
      else  % case 2: "otherwise"
        if trace(Tbd(1:3,1:3))+1 < 1e-5  % case (b): theta = pi; use Eqns 3.58--3.60
           thbd = pi;
           if abs(Tbd(1,1)+1) > 1e-5
	     wbnb = 1/sqrt(2*(1+Tbd(1,1)))*[1+Tbd(1,1); Tbd(2,1); Tbd(3,1)];
           elseif abs(Tbd(2,2)+1) > 1e-5
	     wbnb = 1/sqrt(2*(1+Tbd(2,2)))*[Tbd(1,2); 1+Tbd(2,2); Tbd(3,2)];
           else
	     wbnb = 1/sqrt(2*(1+Tbd(3,3)))*[Tbd(1,3); Tbd(2,3); 1+Tbd(3,3)];
           end
           % now convert that non-bracketed form of w to the bracketed form
           wbd = [0 -wbnb(3) wbnb(2); wbnb(3) 0 -wbnb(1); -wbnb(2) wbnb(1) 0];
        else  % case (c): general case
          thbd = acos((Tbd(1,1)+Tbd(2,2)+Tbd(3,3)-1)/2);
          wbd = 1/2/sin(thbd)*(Tbd(1:3,1:3)-(Tbd(1:3,1:3))');
        end
        % v is calculated according to Eqns 3.91 and 3.92:
        Ginv = eye(3)/thbd - wbd/2 + (1/thbd - cot(thbd/2)/2)*wbd*wbd;
        vbd = Ginv*Tbd(1:3,4);
      end
      % Step 2 of the iterative IK algorithm (above Example 6.1) is to set [Vb] = log(T_bd) = log(T^-1_ab*T_ad)
      Vbb = [wbd*thbd vbd*thbd ; 0 0 0 0];
      %; Now convert to the non-bracketed form Vb of that [Vb]:
      Vb = [Vbb(3,2); Vbb(1,3); Vbb(2,1); Vbb(1:3,4)];
    
      % Update the thetas based on that J and Vb:
      thguess = thguess + pinv(J) * Vb;
    end
    
    theta = thguess;
    
    % Modulate the angles
    thetaMod = mod(theta,2*pi);
    for anglecount = 1:length(thetaMod)
        if thetaMod(anglecount) > pi
            thetaMod(anglecount) = thetaMod(anglecount) - (2*pi);
        else
            thetaMod(anglecount) = thetaMod(anglecount);
        end
    end
    theta = thetaMod;
end

function dist = edist(pos1, pos2)
    dist = sqrt((pos1(1) - pos2(1))^2 + (pos1(2) - pos2(2))^2);
end

% FOR XY MOVEMENT, fixed Z
% convert waypoint Transformation matrices to thetas, given a
% theta guess or theta from previous trajectory
function thetas = executeTrajectory(wayPoints, thetaprev, fixedZ)    
        if size(thetaprev) == [6, 1]
            thetas = thetaprev.';
        else
            thetas = thetaprev;
        end
        
        % Moves the robot through the list of provided waypoints in the
        % specified z plane.
        for i=2:length(wayPoints)
            T = [[-1 0 0 wayPoints(i,1)];
                  [0 1 0  wayPoints(i,2)];
                  [0 0 -1 fixedZ];
                  [0 0 0 1]]; 
            thetas(end+1,:) = IKshell(T, thetas(end,:));
        end
        
        % remove the previous theta that is not part of trajectory but which
        % we used to kick off our IK estimation 
        thetas = thetas(2:end, :);
end

% FOR YZ MOVEMENT, fixed X
function thetas = Z_executeTrajectory(wayPoints, thetaprev, fixedX)
        if size(thetaprev) == [6, 1]
            thetas = thetaprev.';
        else
            thetas = thetaprev;
        end
        
        % Moves the robot through the list of provided waypoints in the
        % specified z plane.
        for i=2:length(wayPoints)
            T = [[-1 0 0 fixedX];
                  [0 1 0  wayPoints(i,1)];
                  [0 0 -1 wayPoints(i,2)];
                  [0 0 0 1]]; 
            thetas(end+1,:) = IKshell(T, thetas(end,:));
        end
        
        % remove the previous theta that is not part of trajectory but which
        % we used to kick off our IK estimation 
        thetas = thetas(2:end, :);
end

% predict the FK
% given thetas, determine fixed-frame to body transformation T_bs
function Tfinal = DH(th)
    % Uses the Denavit-Hartenberg method to determine the final position of
    % the end effector using the joint positions of the robot. 
    % th is a 6x1 matrix describing the robot's joint positions.
    
    % The given dimensions of the robot:
    H1 = 162.5;
    H2 = 99.7;
    L1 = 425;
    L2 = 392.2;
    W1 = 133.3;
    W2 = 259.6;
    
    
    % Since in the D-H formulation, the transformation that occurs at each
    % joint is a product of four simpler transformations (each a pure
    % translation or rotation, with respect to a single coordinate axis),
    % create helper functions for performing those simpler transformations:
    
    function Tret = RX(thet)
        Tret = [1 0 0 0 ; 
            0 cos(thet) -sin(thet) 0; 
            0 sin(thet) cos(thet) 0; 
            0 0 0 1];
    end
    
    function Tret = RY(thet)
        Tret = [cos(thet) 0 sin(thet) 0; 
            0 1 0 0; 
            -sin(thet) 0 cos(thet) 0; 
            0 0 0 1];
    end
    
    function Tret = RZ(thet)
        Tret = [cos(thet) -sin(thet) 0 0; sin(thet) cos(thet) 0 0; 0 0 1 0; 0 0 0 1];
    end
    
    function Tret = TX(dist)
        Tret = [1 0 0 dist; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    end
    
    function Tret = TY(dist)
        Tret = [1 0 0 0; 0 1 0 dist; 0 0 1 0; 0 0 0 1];
    end
    
    function Tret = TZ(dist)
        Tret = [1 0 0 0; 0 1 0 0; 0 0 1 dist; 0 0 0 1];
    end
    
    % Now you can use those functions and the D-H table to calculate the
    % % transformation matrix for each successive joint relative to the last:
    
    T1 = RZ(th(1))*TZ(H1)*TX(0)*RX(-pi/2);
    T2 = RZ(th(2))*TZ(0)*TX(L1)*RX(0);
    T3 = RZ(th(3))*TZ(0)*TX(L2)*RX(0);
    T4 = RZ(th(4))*TZ(W1)*TX(0)*RX(-pi/2);
    T5 = RZ(th(5))*TZ(H2)*TX(0)*RX(pi/2);
    T6 = RZ(th(6)+pi)*TZ(W2)*TX(0)*RX(0);
    
    % Next calculate the final transformation which is the result of
    % performing the six separate transformations in succession:
    Tfinal = T1*T2*T3*T4*T5*T6;
%     Tfinal = [Tfinal(1,4), Tfinal(2,4), Tfinal(3,4)];   % x,y,z
end
