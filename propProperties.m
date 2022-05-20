% property file propeller and pointcloud hard coded constantss

% User chosen Name for the Propeller
propName = 'Tmotor_12x4';

propDiameter = 12; % inches
trueRadius = 6; % inches 
pitch = 4; % inches
bladeDirection = -1; % 1 CW,-1 CCW Used for computing twist correctly later
propHubThickness = .275; % in

% Coordinate System Alignment Seeds
seeds = [-0.324407339096069,-0.317308425903320,-0.507158041000366,-0.603542327880859,-0.728489398956299;
    1.82155227661133,1.80024075508118,1.63609004020691,1.73359072208405,1.89328360557556;
    2.89067220687866,3.14666748046875,2.95345520973206,3.24981045722961,3.01777005195618];
