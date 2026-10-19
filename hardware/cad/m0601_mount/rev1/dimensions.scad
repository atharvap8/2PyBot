// Millimetres. Current STL assembly: X lateral, Y front->back, Z up;
// bottom PCB solder face is Z=0. Chassis anchor dimensions remain estimates.

/* [Chassis - confirm on the physical robot] */
board_width = 150;
motor_anchor_x = 23;
motor_anchor_y = 45;
strut_pitch_x = 30;
strut_pitch_y = 30;
axle_drop = 65;
tray_opening = 44;
tray_fit_per_side = 0.30;
tray_ledge_inner_x = 2.2;
tray_ledge_lower_z = -7;
tray_ledge_clearance = 0.4;

/* [Adapter] */
frame_depth = 10;
frame_rail = 10;
stator_face_x = 24;
socket_lip_x = 11;
face_plate_thickness = 6;
socket_outer_diameter = 34;
motor_clock_deg = 0;
cable_direction = 1; // 1 rear (+Y), -1 front
cable_channel_height = 12;
chassis_clearance_d = 3.4;
nut_across_flats = 5.8;
nut_pocket_height = 2.7;
nut_roof_depth = 3.5;

/* [M0601 - manufacturer drawing] */
motor_bolt_circle = 15.20;
motor_bolt_clearance = 2.90;
motor_thread_depth = 5;
stator_nominal_diameter = 19;
stator_max_diameter = 19.05;
stator_nominal_flat_radius = 8;
stator_max_flat_radius = 8.15;
stator_projection = 15;
stator_projection_min = 14.45;
stator_projection_max = 15.95;
overall_width = 65.50;
overall_width_max = 66;
wheel_diameter = 100.70;
wheel_diameter_min = 99.20;
wheel_diameter_max = 102.20;

/* [Print fit - adjust after printing the motor gauge] */
socket_diameter_clearance = 0.40;
socket_flat_clearance = 0.25;

/* [Hidden] */
$fn = 96;
epsilon = 0.03;
frame_size = tray_opening-2*tray_fit_per_side;
frame_x0 = motor_anchor_x-frame_size/2;
frame_x1 = motor_anchor_x+frame_size/2;
frame_y0 = motor_anchor_y-frame_size/2;
frame_y1 = motor_anchor_y+frame_size/2;
axle_z = -axle_drop;
cap_back_x = stator_face_x+face_plate_thickness;
socket_depth = stator_face_x-socket_lip_x;
socket_diameter = stator_max_diameter+socket_diameter_clearance;
socket_flat_radius = stator_max_flat_radius+socket_flat_clearance;
anchor_x = [motor_anchor_x-strut_pitch_x/2,motor_anchor_x+strut_pitch_x/2];
anchor_y = [motor_anchor_y-strut_pitch_y/2,motor_anchor_y+strut_pitch_y/2];
