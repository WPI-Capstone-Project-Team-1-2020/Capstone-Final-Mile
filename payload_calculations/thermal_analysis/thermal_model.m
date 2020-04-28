% Create the thermal model
model = createpde('thermal', 'transient');

% Import the object geometry and assign to model
geo = importGeometry(model, 'Thermal_Sim_Model_With_Hole.stl');
outside_faces = [7 8 9 10 11 12];
blood_bag_blob_faces = [1 2 3 4 5 6];

% Show the model
pdegplot(model, 'CellLabels', 'on', 'FaceAlpha', 0.5, 'FaceLabels', 'on')

styrofoam_faces = [9 10 11];
% Thermal properties of the Styrofoam Material
thermal_cond = 0.033; %(W/m)*K
mass_density = 50; % kg/m^3
specific_heat = 1131; % J/kg * K
thermalProperties(model,'Cell', 1,...
                        'ThermalConductivity', thermal_cond,...
                        'MassDensity', mass_density,...
                        'SpecificHeat',specific_heat);
                    
% Thermal properties of the Blood Bag Blob
% https://www.engineeringtoolbox.com/water-liquid-gas-thermal-conductivity-temperature-pressure-d_2012.html
thermal_cond = 0.58; %(W/m)*K
mass_density = 1000; % kg/m^3
specific_heat = 4191; % J/kg * K
thermalProperties(model,'Cell', 2,...
                        'ThermalConductivity', thermal_cond,...
                        'MassDensity', mass_density,...
                        'SpecificHeat',specific_heat);
                     
% Thermal properties of the Aluminum Material (Inner Reflective Layer)
% thermal_cond = 205; %(W/m)*K
% mass_density = 2700; % kg/m^3
% specific_heat = 910; % J/kg * K
% thermalProperties(model,'Cell', aluminum_faces,...
%                         'ThermalConductivity', thermal_cond,...
%                         'MassDensity', mass_density,...
%                         'SpecificHeat',specific_heat);

% Set inital conditions to be used when solving for the transient response
% of the thermal model
internal_temp = 2 + 273; % Temperature of Blood Bag
% Blood bag cannot go about 273+6

% Set the thermal inital conditions
thermalIC(model, internal_temp, 'Cell', [1, 2]);

% Apply temperature on the boundary of the payload container as a constant
% heat sink so we can observe how quickly our payload absorbs heat
max_amb_temp = 40 + 273;
model.StefanBoltzmannConstant = 5.670373E-8;
thermalBC(model,'Face',outside_faces,...
                       'Emissivity',0.60,...
                       'AmbientTemperature',max_amb_temp)

% Generate Model Mesh
generateMesh(model);

% Solve for the transient thermal analysis and transient heat flux
flight_time_max = 12 * 60;
flight_duration = 0:flight_time_max;
thermal_result = solve(model, flight_duration);
[qx,qy,qz] = evaluateHeatFlux(thermal_result);
blood_bag_face = evaluateHeatRate(thermal_result, 'Face', 1);
container_face = evaluateHeatRate(thermal_result, 'Face', 12);

%% Plot the Cell Heat Rates
blood_bag_heat_rate = figure;
plot(flight_duration, blood_bag_face);
title('Heat Rate of the blood bag during flight duration')
xlabel('Time [s]')
ylabel('Heat Flow Rate [J] (- is into the body)')
saveas(blood_bag_heat_rate, 'Bloog_Bag_Heat_Rate.png')
close

contianer_heat_rate = figure;
plot(flight_duration, container_face);
title('Heat Rate of an external face of the payload container during flight duration')
xlabel('Time [s]')
ylabel('Heat Flow Rate [J](- is into the body)')
saveas(blood_bag_heat_rate, 'Container_Heat_Rate.png')
close

%% Create a movie of the transient thermal analysis
for n = 1:length(thermal_result.SolutionTimes) - 1
    pdeplot3D(model, 'ColorMapData', thermal_result.Temperature(:,n));
    view(10,3);
    F(n) = getframe(gcf);
    drawnow
end

% Write the frames into a video
writerObj = VideoWriter('thermal_transient_response.avi');
writerObj.FrameRate = 10;
open(writerObj);
for i = 1:length(F)
    frame = F(i);
    writeVideo(writerObj, frame);
end
close(writerObj);


%% Create a movie of the heat flux analysis
for n = 1:length(thermal_result.SolutionTimes) - 1
    pdeplot3D(model, 'FlowData', [qx(:,n),qy(:,n),qz(:,n)]);
    view(10,3);
    F(n) = getframe(gcf);
    drawnow
end

% Write the frames into a video
writerObj = VideoWriter('heat_flux_transient_response.avi');
writerObj.FrameRate = 10;
open(writerObj);
for i = 1:length(F)
    frame = F(i);
    writeVideo(writerObj, frame);
end
close(writerObj);

