% Moments of inertia of a solid rectangular cuboid and a solid cylinder
% https://en.wikipedia.org/wiki/List_of_moments_of_inertia

body_mass = 10.0;
body_width = 0.3;
body_depth = 0.3;
body_length = 0.8;

body_ixx = body_mass / 12 * (body_width^2 + body_length^2)
body_iyy = body_mass / 12 * (body_length^2 + body_depth^2)
body_izz = body_mass / 12 * (body_width^2 + body_depth^2)


wheel_mass = 1.0;
wheel_radius = 0.2;
wheel_thickness = 0.1;

wheel_ixx = wheel_mass / 12 * (3 * wheel_radius^2 + wheel_thickness^2)
wheel_iyy = wheel_mass / 12 * (3 * wheel_radius^2 + wheel_thickness^2)
wheel_izz = wheel_mass / 2 * wheel_radius^2
