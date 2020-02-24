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
in_to_mm = 25.4;
top_width = fw - (bpw / 2);
bottom_width = top_width + bph * tand(leg_pitch_angle);

outside_height = bph / cosd(leg_pitch_angle);
inside_height = bph;

length = body_length;

payload_volume = ((top_width * inside_height * length) * in_to_mm^3 / 1000)...
    + (((bottom_width - top_width) * inside_height * 0.5 * length) * in_to_mm^3 / 1000);

%% Blood Bag dimensions in inches
blood_bag_volume = 450; % milliliters

bbl = 6 * in_to_mm; 
bbw = 8 * in_to_mm;
bbh = 0.6 * in_to_mm;

bb_volume_estimate = (bbl*bbw*bbh) / 1000;

%% Draw the calculated dimensions
% Making assumption about the pixels per inch in the image
pixels_per_inch = 6;

% To draw them on the image, we need to get the pixels per inch assumption
f1 = figure;
f2 = figure;

figure(f1);
hold on
title('Payload Cross-section Area in Inches')
% Draw the payload container cross section
plot([0, bottom_width, bottom_width, top_width, 0],...
    [0, 0, inside_height, inside_height, 0])


figure(f2)
hold on
% Draw the top side
imshow('Matrice_200_Front_View.png');
origin_x = 200;
origin_y = 145;
line([origin_x, origin_x + (top_width * pixels_per_inch), origin_x + (top_width * pixels_per_inch), origin_x - ((bottom_width - top_width) * pixels_per_inch), origin_x],...
    [origin_y, origin_y, origin_y + (inside_height * pixels_per_inch), origin_y + (inside_height * pixels_per_inch), origin_y],...
    'Color', 'r', 'LineWidth', 3)



