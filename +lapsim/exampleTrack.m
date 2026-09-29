function track = exampleTrack()
%EXAMPLETRACK User race distance with historical approximate slow corner.
% Explicit row lengths are the only authority for lap distance: 3832.5 m.
% Only the final legacy slow corner is constrained. Other curvature unknown.
name = ["Main section"; "Legacy turn 13"; "Final section"];
length_m = [3256; 45; 531.5];          % Final straight adjusted to user 15.33 km / 4
grade = [0; 0; 0];                    % rise/run, positive uphill; not degrees
radius_m = [Inf; 45/(3*pi/4); Inf];    % 45 m / 135 deg, approximate centerline radius
speed_limit_mps = [Inf; 7; Inf];
track = table(name,length_m,grade,radius_m,speed_limit_mps);
end
