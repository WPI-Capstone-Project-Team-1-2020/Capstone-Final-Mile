%% Known Values

% Can subtract from values with motor height assumption
motor_height = 1.0;

% Can subtract from dimensions with propellor inclusion assumption. Know
% propeller length is 4.0 inches
prop_length = 4.0;

% Unfolded Dimensions
ufl = 34.9 - prop_length / 2;
ufw = 34.6 - prop_length / 2;
ufh = 14.9 - motor_height;

% Folded Dimensions
fl = 28.2;
fw = 8.7;
fh = 9.3 - motor_height;

%% Assumptions needed to be made
bpl_percentage = 0.7;
bpw_percentage = 0.75;
bph_percentage = 0.80;
gps_antenna_height = 2.0; % Inches
arm_swing_angle = 60;
leg_pitch_angle = 27;

%% Calculations for Drone Dimensions based on Assumptions

% Calculates the Battery Pack Height (bhp) based on a percentage of the total 
% folded height
bph = fh * bph_percentage;

% Calculates the arm swing angle (Optional)
% arm_swing_angle = acosd((arm_length - (fl - ufl)) / arm_length);

% Calculates the Battery Pack Length (bpl) based on the following parameters:
% blp_percentage - Battery pack length % of total length
% arm_swing_angle - Degree the arm sweeps out to when unfolding
arm_length = (ufw - fw) / (2 * sind(arm_swing_angle));
body_length = fl - arm_length;
bpl = body_length * bpl_percentage;

% Calculates the Battery Pack Width (bpw) based on percentage of the total
% folded width
bpw = fw * bpw_percentage;

% Calculates the leg height based on the following parameters:
% gps_antenna_height - gps_antenna_height in the unfolded position
% leg_pitch_angle - Degree at which the leg kicks out to when attached
height_wo_antenna = ufh - gps_antenna_height;
leg_length = height_wo_antenna / cosd(leg_pitch_angle);

%% Calculation of payload space dimensions
% The payload takes the shap of a square and right triangle. This is where
% we calculate its dimensions

% Reduce bph to reduce payload container weight
bph = bph * 0.80;

in_to_mm = 25.4;
top_width = fw - (bpw / 2);
bottom_width = top_width + bph * tand(leg_pitch_angle);

outside_height = bph / cosd(leg_pitch_angle);
inside_height = bph;

length = body_length;

% Reduced length of payload container
%length = 6;

payload_volume = ((top_width * inside_height * length) * in_to_mm^3 / 1000)...
    + (((bottom_width - top_width) * inside_height * 0.5 * length) * in_to_mm^3 / 1000);
disp(['Build space volume is ', num2str(payload_volume), ' ml.'])

%% Blood Bag dimensions in inches
blood_bag_volume = 450; % milliliters

bbl = 6 * in_to_mm; 
bbh = 8 * in_to_mm;
bbw = 0.6 * in_to_mm;

bb_volume_estimate = (bbl*bbw*bbh) / 1000;


%% Draw the calculated dimensions
% Making assumption about the pixels per inch in the image
pixels_per_inch = 6;

% To draw them on the image, we need to get the pixels per inch assumption
f1 = figure;
%f2 = figure;

% Visualize the payload cross sectional area
figure(f1);
hold on
title('Payload Cross-section Area in Inches')

% The payload container has 4 corners, so lets store them here
p1 = [0, 0];
p2 = [bottom_width, 0];
p3 = [bottom_width, inside_height];
p4 = [bottom_width - top_width, inside_height];
points = [p1; p2; p3; p4];

% Draw the Outer Payload container cross section
% plot([points(:,1); points(1,1)],...
%      [points(:,2); points(1,2)]);

% Add a thickness layer to the normal faces. Just a simple addition or
% subtraction
thickness = 1; % Inches

% X points
points(2:3,1) = points(2:3,1) - thickness;

% Y points
points(1:2,2) = points(1:2,2) + thickness;
points(3:4,2) = points(3:4,2) - thickness;

% Line equation of the angled face to add thickness
points(1,1) = thickness + (points(4,1) / points(4,2)) * points(1,2);
points(4,1) = thickness + (points(4,1) / points(4,2)) * points(4,2); %points(4,1) + thickness;

% Draw the inner layer of the payload container
% plot([points(:,1); points(1,1)],...
%      [points(:,2); points(1,2)]);
 
% Recalculate payload volume
top_width = points(3,1) - points(4,1);
bottom_width = points(2,1) - points(1,1);
inside_height = points(3,2) - points(2,2);

payload_volume_with_thickness = ((top_width * inside_height * (length - thickness)) * in_to_mm^3 / 1000)...
    + (((bottom_width - top_width) * inside_height * 0.5 * (length - thickness)) * in_to_mm^3 / 1000);
disp(['Volume in payload with ', num2str(thickness), ' inch thickness to carry 2 blood bags is ', num2str(payload_volume_with_thickness), ' ml.']);
disp(['The volume 2 blood bags occupy is ', num2str(blood_bag_volume * 2), ' ml.']);

% Calculate the payload weight
styrofoam_density = 0.05; % g/ml
g_to_lb = 0.00220462;

payload_container_weight = (payload_volume - payload_volume_with_thickness) * styrofoam_density * g_to_lb;
disp(['The total weight of both payload containers is ', num2str(payload_container_weight * 2), ' pounds']);

num_blood_bags = 4;
blood_density = 1.125; % g/ml

blood_weight = num_blood_bags * bb_volume_estimate * blood_density * g_to_lb;
disp(['The total weight of blood to be carried is ', num2str(blood_weight), ' lbs for ', num2str(num_blood_bags), ' blood bags.']);

% Add the weight of the onboard computer
added_onboard_computer = 230 * g_to_lb;
disp(['The weight of the onboard computer is ', num2str(added_onboard_computer), ' lbs']);

% Calculate total payload weight
total_payload_weight = payload_container_weight + blood_weight + added_onboard_computer;
disp(['The total payload weight for the drone is ', num2str(total_payload_weight), ' lbs or ',...
    num2str(total_payload_weight * 0.453592), ' kgs.'])

% Final payload dimension
top_width = fw - (bpw / 2);
bottom_width = top_width + bph * tand(leg_pitch_angle);

outside_height = bph / cosd(leg_pitch_angle);
inside_height = bph;

disp(['The payload container has the following dimensions:', newline, 'length: ', num2str(length),...
    ' inches', newline, 'height: ', num2str(inside_height),...
    ' inches', newline, 'bottom width: ', num2str(bottom_width),...
    ' inches', newline, 'top width: ', num2str(top_width),...
    ' inches']);

 
% Draw the blood bags into the payload area, starting up against the inner
% wall
% Blood Bag 1
% bbl = 6; 
% bbh = 8;
% bbw = 0.6;
% 
% origin_x = points(2,1) - bbw;
% origin_y = points(2,2);
% 
% blood_bag1_x = [origin_x, origin_x + bbw, origin_x + bbw, origin_x, origin_x];
% blood_bag1_y = [origin_y, origin_y, origin_y + bbh, origin_y + bbh, origin_y];
% blood_bag1_points = [blood_bag1_x ; blood_bag1_y]';
% 
% plot([blood_bag1_points(:,1); blood_bag1_points(1,1)],...
%      [blood_bag1_points(:,2); blood_bag1_points(1,2)]);

% % Blood Bag 2
% origin_x = points(2,1) - blood_bag1_x(4);
% origin_y = points(2,2) - blood_bag1_y(4);
% 
% blood_bag2_x = [origin_x, origin_x + top_width, origin_x + top_width, origin_x - (bottom_width - top_width), origin_x];
% blood_bag2_y = [origin_y, origin_y, origin_y + inside_height, origin_y + inside_height, origin_y];
% blood_bag2_points = [blood2_bag_x ; blood_bag2_y];
% 
% plot([blood_bag2_points(:,1); blood_bag2_points(1,1)],...
%      [blood_bag2_points(:,2); blood_bag2_points(1,2)]);

%Visualize the Payload on the drone
% figure(f2)
% hold on
% % Draw the top side
% imshow('Matrice_200_Front_View.png');
% origin_x = 200;
% origin_y = 145;
% line([origin_x, origin_x + (top_width * pixels_per_inch), origin_x + (top_width * pixels_per_inch), origin_x - ((bottom_width - top_width) * pixels_per_inch), origin_x],...
%     [origin_y, origin_y, origin_y + (inside_height * pixels_per_inch), origin_y + (inside_height * pixels_per_inch), origin_y],...
%     'Color', 'r', 'LineWidth', 3)



