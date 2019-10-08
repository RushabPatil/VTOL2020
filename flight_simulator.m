function t_flight = flight_simulator(horz_motor_info,Cl,Cd,S,M_L,V_level,E_TOL,E_transition,E_cap,margin,plot_trigger)

% simulate overal aircraft flight profile
% flight_simulator(horz_motor_info,Cl,Cd,S,M_L,V_level,E_TOL,E_transition,E_cap,margin,plot_trigger)
% horz_motor_info: manufacturer data on motor relations between current,
% thrust, and speed
% Cl: airfoil coefficient of lift at trim angle of attack
% Cd: aircraft coefficient of drag at trim angle of attack
% S: airfoil wetted wing area, m^2
% M_L: loaded mass of aircraft, kg
% V_level: trim velocity, m s^-1
% E_TOL: energy consumed for takeoff and landing, W-hr
% E_transition: energy reserved for transitions, W-hr
% E_cap: total capacity of battery, W-hr
% margin: allowable fraction of total capacity for flight period plus takeoff and landing
E_consumed = E_TOL + E_transition;

% constants
g = 9.81;
W_L = M_L*g;
rho = 1.225;
Q = 0.5*rho*V_level^2;

% pull motor data from total matrix
motor_data = [0 0; horz_motor_info(:,3) g*0.001*horz_motor_info(:,2)];
% isolate maximum thrust motor can deliver
Tmax = max(motor_data(:,2));
% assume power required for autonomy and other functions
P_autonomy = 20;    % W

% airfield corner coordinates (x,y)
f2m = 0.3048;   % conversion from feet to meters
x_L = 900*f2m;  % width of flight area, m
y_L = 200*f2m;  % height of flight area, m
% define coordinates of permitted flight area, with the origin in the
% bottom left corner
Corners = [0 0; 0 y_L; x_L y_L; x_L 0];
% define coordinates of takeoff + landing area
Landing_Corners = [350 0; 350 60; 550 60; 550 0];
% define headings that correspond to outward facing normal of each edge of
% flight area
Edge_norms = wrapTo2Pi(pi*(1:-0.5:-0.5));
dt = 0.01;  % time step, s
x0 = [400*f2m; 0]; % initial positon, [x0 (m); y0 (m)]

heading = pi;
%% straight line motion
% aircraft will only follow a linear path when not turning
D_straight = Q*S*Cd;
% apply trim condition T = D, then convert this to a power requirement by
% linearly interpolating in the manufacturer data
%W_straight = interp1q(motor_data(:,2),motor_data(:,1),D_straight);
% The above should work, but to check our answer with the previous
% calculation, use power with bigger drag
W_straight = 115;

% add autonomy power requirements for net straight line motion
W_snet = W_straight + P_autonomy;
%% turning motion
% aircraft will only perform 90 degree turns and follow a clockwise path
% around the field

% perform smallest possible turn
CL_max = 1.56;              % airfoil max coefficient of lift
c=.3048;                    %m airfoil chord
e=.8;                       %Oswald Efficency
b=W_L/(.5*Cl*rho*V_level^2*c);    %m wingspan
AR=b^2/(b*c);               % Aspect Ratio
K = (4/3)/(pi()*e*AR);      %Drag polar
Cd_i_max = CL_max^2/(pi*AR*e);
Cd0 = Cl^2/(pi*AR*e);
% use this max lift to calculate minimum turning radius

% the drag coefficient at alpha_CLmax is known. Calculate drag of
% alpha_CLmax, setting it equal to turning thrust.
D_max = Q*S*Cd_i_max;
% linearly interpolate to yield motor power during turn
W_turn = interp1q(motor_data(:,2),motor_data(:,1),D_max);
% minimum turn radius
R_min = Radius_of_turn(V_level,W_L,S,Tmax,Cd0,CL_max,e,AR,K);
dhead_max = V_level/R_min;

% add autonomy power to result total power during turn
W_tnet = W_turn + P_autonomy;
% initialize loop
x(:,1) = x0;    % compiled list of aircraft positions
turning = false;    % boolean indicating if aircraft needs to turn
goal_heading = heading; % target heading of aircraft (only relevant during turning)
goals(1) = goal_heading;    % compiled list of goal headings
% actual loop
i = 0;
t(1) = 0;
W_overall = [];
Turn_count = 0;
while E_consumed < E_cap*margin
    i = i + 1;
    if turning
        % simulate turning
        dheading = -dhead_max;
        
        heading(i+1) = wrapTo2Pi(dheading*dt + heading(i));
        dE = W_tnet*dt;
        W_overall(i) = W_tnet;
        if abs(goal_heading - heading(i)) <= 0.2
            heading(i+1) = goal_heading;
            Turn_count = Turn_count + 1;
            turning = false;
        end
    else
        heading(i+1) = wrapTo2Pi(heading(i));
        % simulate straight-line flight
        if min(edge_check(x(:,i),Corners,heading(i+1),Edge_norms)) <= R_min
            goal_heading = wrapTo2Pi(heading(i+1) - pi/2);
            turning = true;
        end
        dE = W_snet*dt;
        W_overall(i) = W_snet;
    end
    dx = V_level*dt*[cos(heading(i)); sin(heading(i))];
    t(i + 1) = t(i) + dt;
    x(:,i+1) = x(:,i) + dx;
    goals(i+1) = wrapTo2Pi(goal_heading);
    E_consumed = E_consumed + dE/3600;
end
heading = wrapTo2Pi(heading);
% plot components of position vs. time
if plot_trigger
    % plot positions vs. time
    figure
    plot(t,x)
    legend('x(t)','y(t)')
    xlabel('Time (s)')
    ylabel('Position (m)')

    % plot heading vs. time
    figure
    plot(t,heading,t,goals,'--r')
    title('Variation of Heading During Simulation')
    legend('Current Heading (rad)','Desired Heading (rad)')
    xlabel('Time (s)')
    ylabel('Angle (rad)')

    % plot trajectory in space
    figure
    plot(x(1,:)/f2m,x(2,:)/f2m)
    title("Estimated Endurance Flight Path")
    xlabel("x position (m)")
    ylabel("y position (m)")
    % ensure axes are equally spaced
    daspect(ones(1,3))
    hold on
    % mark start and end points
    plot(x(1,1)/f2m,x(2,1)/f2m,'or')
    plot(x(1,end)/f2m,x(2,end)/f2m,'ok')
    % add borders of takeoff/landing zone and total flight area
    plot(Corners(:,1)/f2m,Corners(:,2)/f2m,'--k',Landing_Corners(:,1),Landing_Corners(:,2),'--g')
    hold off
end

% print relevant stuff
t_flight = floor(t(end));
fprintf('The total predicted flight time is %i seconds\n',t_flight)
fprintf('The number of turns completed is %i\n',Turn_count)

%% Functions

function distances = edge_check(current_position,Corners,heading_i,Edge_norms)
    % compare current heading to outward facing normal of each side of the
    % field
    rel_head = wrapTo2Pi(heading_i) - wrapTo2Pi(Edge_norms);
    % check distance to each of the 4 sides of the permitted flight zone
    for index = 1:size(Corners,1)
        % only check trajectory relative to the two normals less than 90
        % degrees. This avoids a negative cosine being returned, as
        % distance must be positive
        if abs(rel_head(index)) < (pi/2)
            % ind_i selects which dimension (along x- or y-) is being
            % currently measured. As the normal to sides 1 and 3 is
            % parallel to the x-axis ind_i(1) = ind_i(3) = 1. Likewise,
            % ind_i(2) = ind_i(4) = 2 as the normal to these sides is
            % parallel to the y-axis.
            ind_i = mod(index+1,2)+1;
            % project the actual 1-d distance to the side on the path
            % parallel to the current heading. Thanks to this, a trajectory
            % not perpendicular to a side will appear farther from it, as
            % the plane will not need to turn as soon to stay in the field.
            distances(index,:) = abs(current_position(ind_i) - Corners(index,ind_i))/cos(rel_head(index));
        else
            % if the aircraft is not facing in the general direction of a
            % corner, treat the distance to the corner as very large
            distances(index,:) = 100;
        end
    end
    
end
end