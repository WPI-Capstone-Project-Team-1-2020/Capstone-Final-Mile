% Create the thermal model
model = createpde('thermal', 'transient');

% Import the object geometry and assign to model
geo = importGeometry(model, 'Box.stl');
inside_faces = [1 2 3 4 10];
outside_faces = [5 6 7 8 9 11];

% Show the model
pdegplot(model, 'CellLabels', 'on', 'FaceAlpha', 0.5, 'FaceLabels', 'on')

% Thermal properties of the Payload Material (Styrofoam)
thermal_cond = 0.033; %(W/m)*K
mass_density = 50; % kg/m^3
specific_heat = 1131; % J/kg * K
thermalProperties(model, 'ThermalConductivity', thermal_cond,...
                         'MassDensity', mass_density,...
                         'SpecificHeat',specific_heat);

% Apply temperature on the boundary of the payload container as a constant
% heat sink so we can observe how quickly our payload absorbs heat
max_amb_temp = 40 + 273;
%thermalBC(model, 'Face', outside_faces, 'Temperature', max_amb_temp);
%thermalBC(model, 'Face', outside_faces, 'HeatFlux', max_amb_temp);
model.StefanBoltzmannConstant = 5.670373E-8;
thermalBC(model,'Face',outside_faces,...
                       'Emissivity',0.60,...
                       'AmbientTemperature',max_amb_temp)

% Set inital conditions to be used when solving for the transient response
% of the thermal model
internal_temp = 2 + 273; % Temperature of Blood Bag
% Blood bag cannot go about 273+6

%thermalIC(model, internal_temp, 'Face', inside_faces);
%thermalIC(model, max_amb_temp - 1, 'Face', outside_faces);
temp = thermalIC(model, internal_temp);

% Generate Model Mesh
generateMesh(model);

% Solve for the transient thermal analysis
flight_duration = 6 * 360;
result = solve(model, 1:10:flight_duration);


%% Create a movie of the transient thermal analysis
for n = 1:length(result.SolutionTimes) - 1
    pdeplot3D(model, 'ColorMapData', result.Temperature(:,n));
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


