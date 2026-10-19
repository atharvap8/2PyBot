// M0601 strut-replacement bracket. One rigid fork per side, four original
// PCB anchors, captive M3 nuts, keyed stator socket and three M2.5 bolts.
// Export parts with -D 'part="mount"' -D 'side="L"' -D 'layout="print"'.
include <dimensions.scad>
part = "mount"; // mount, chassis_gauge, motor_gauge, assembly, envelope, stator, parameters
side = "L";
layout = "print"; // print, assembly
maximum_envelope = false;

assert(strut_pitch_x>=28 && strut_pitch_x<=32,"Anchor X pitch outside this frame/nut-pocket design");
assert(strut_pitch_y>=28 && strut_pitch_y<=32,"Anchor Y pitch outside this frame/nut-pocket design");
assert(socket_lip_x>stator_face_x-stator_projection_min,"Socket lip must clear rotating wheel face");
assert(frame_depth<axle_drop-wheel_diameter_max/2,"Top frame intrudes into maximum wheel height");
assert(socket_depth>5 && face_plate_thickness>=5,"Socket/face plate too shallow");
assert(cable_direction==1 || cable_direction==-1,"Cable direction must be rear/front");

// Local 2D XY becomes assembly YZ; extrusion is along the wheel axle (X).
module along_axle(x,length,y=motor_anchor_y,z=axle_z) {
    multmatrix([[0,0,1,x],[1,0,0,y],[0,1,0,z],[0,0,0,1]])
        linear_extrude(length) children();
}
// Drawing's 3x8 EQS: three flats, each 8 mm from axis, 120 degrees apart.
// Their normals lie BETWEEN the three bolt directions.
module keyed_profile(diameter,flat_radius) {
    rotate(motor_clock_deg) intersection() {
        circle(d=diameter);
        for(angle=[30,150,270]) rotate(angle)
            translate([-50,-50]) square([50+flat_radius,100]);
    }
}
module bolt_profile() {
    for(angle=[90,210,330])
        rotate(angle+motor_clock_deg) translate([motor_bolt_circle/2,0])
            circle(d=motor_bolt_clearance);
}
module frame() {
    translate([motor_anchor_x,motor_anchor_y,-frame_depth]) linear_extrude(frame_depth)
        difference() {
            offset(r=2) offset(delta=-2) square([frame_size,frame_size],center=true);
            square([frame_size-2*frame_rail,frame_size-2*frame_rail],center=true);
        }
}
module web(y) {
    // Front/back webs have a wide frame root and narrow toward the axle.
    // X>=socket_lip_x below the top frame: rotating wheel envelope stays clear.
    translate([0,y,0]) rotate([90,0,0]) linear_extrude(frame_rail)
        polygon([[socket_lip_x,-frame_depth+epsilon],
                 [frame_x1,-frame_depth+epsilon],
                 [frame_x1-7,axle_z+21],
                 [cap_back_x,axle_z-socket_outer_diameter/2],
                 [socket_lip_x,axle_z-socket_outer_diameter/2]]);
}
module anchor_cutouts() {
    for(x=anchor_x) for(y=anchor_y) {
        translate([x,y,-frame_depth-1]) cylinder(d=chassis_clearance_d,h=frame_depth+2);
        translate([x,y,-nut_roof_depth-nut_pocket_height])
            rotate([0,0,30]) cylinder(d=nut_across_flats/cos(30),h=nut_pocket_height,$fn=6);
        // Side-load standard M3 hex nut, then fasten from above the PCB.
        rear = y>motor_anchor_y;
        slot_y = rear ? y : frame_y0-1;
        slot_length = rear ? frame_y1-y+1 : y-frame_y0+1;
        translate([x-nut_across_flats/2,slot_y,-nut_roof_depth-nut_pocket_height])
            cube([nut_across_flats,slot_length,nut_pocket_height]);
    }
}
module left_body() {
    difference() {
        union() {
            frame();
            web(frame_y0+frame_rail);
            web(frame_y1);
            along_axle(socket_lip_x,cap_back_x-socket_lip_x) circle(d=socket_outer_diameter);
        }
        anchor_cutouts();
        // Current split-base STL has a PCB support ledge above the motor
        // opening. Relieve the upper outboard rail rather than cutting the tray.
        if(tray_ledge_inner_x+tray_ledge_clearance>frame_x0)
            translate([frame_x0-1,frame_y0-1,tray_ledge_lower_z-tray_ledge_clearance])
                cube([tray_ledge_inner_x+tray_ledge_clearance-frame_x0+1,
                      frame_size+2,-tray_ledge_lower_z+tray_ledge_clearance+1]);
        along_axle(socket_lip_x-epsilon,socket_depth+epsilon)
            keyed_profile(socket_diameter,socket_flat_radius);
        // Short lead-in, leaving the nominal keyed seat deeper in the socket.
        along_axle(socket_lip_x-epsilon,.7+epsilon)
            keyed_profile(socket_diameter+.6,socket_flat_radius+.3);
        along_axle(stator_face_x-epsilon,face_plate_thickness+2*epsilon) bolt_profile();
        // Generous open cable exit along the socket; the six-mm end plate stays intact.
        cable_y0 = cable_direction>0 ? motor_anchor_y+5 : frame_y0-1;
        cable_y1 = cable_direction>0 ? frame_y1+1 : motor_anchor_y-5;
        translate([socket_lip_x-epsilon,cable_y0,axle_z-cable_channel_height/2])
            cube([socket_depth+epsilon,cable_y1-cable_y0,cable_channel_height]);
        // Cable tie slot through rear/front web, accessible before/after assembly.
        tie_y = cable_direction>0 ? frame_y1-frame_rail-1 : frame_y0-1;
        translate([cap_back_x+1,tie_y,axle_z+23]) cube([3.5,frame_rail+2,10]);
    }
}
module mount_assembly(which) {
    difference() {
        if(which=="R") translate([board_width,0,0]) mirror([1,0,0]) left_body();
        else left_body();
        // Engrave side AFTER mirroring so text is not reversed.
        translate([which=="R"?board_width-29:29,frame_y0+.65,-30])
            rotate([90,0,0]) linear_extrude(.8)
                text(which,size=7,font="Liberation Sans:style=Bold",halign="center",valign="center");
    }
}
module printable_mount(which) {
    xmin = which=="R" ? board_width-frame_x1 : frame_x0;
    // Large front web on the bed. Supports needed for horizontal socket ceiling.
    translate([-xmin,0,-frame_y0]) rotate([90,0,0]) mount_assembly(which);
}
module chassis_gauge() {
    difference() {
        linear_extrude(2.4) difference() {
            offset(r=2) offset(delta=-2) square([frame_size,frame_size]);
            translate([8,8]) square([frame_size-16,frame_size-16]);
        }
        for(x=anchor_x) for(y=anchor_y)
            translate([x-frame_x0,y-frame_y0,-epsilon]) cylinder(d=chassis_clearance_d,h=2.4+2*epsilon);
    }
}
module motor_gauge() {
    difference() {
        cylinder(d=socket_outer_diameter,h=face_plate_thickness+socket_depth);
        translate([0,0,face_plate_thickness]) linear_extrude(socket_depth+epsilon)
            keyed_profile(socket_diameter,socket_flat_radius);
        linear_extrude(face_plate_thickness+epsilon) bolt_profile();
        // Same radial cable slot as the main mount, face plate unchanged.
        if(cable_direction>0) translate([5,-cable_channel_height/2,face_plate_thickness])
            cube([socket_outer_diameter,cable_channel_height,socket_depth+epsilon]);
        else translate([-socket_outer_diameter-5,-cable_channel_height/2,face_plate_thickness])
            cube([socket_outer_diameter,cable_channel_height,socket_depth+epsilon]);
    }
}
module wheel_envelope(which,maximum=false) {
    face = stator_face_x-(maximum?stator_projection_min:stator_projection);
    outside = stator_face_x-(maximum?overall_width_max:overall_width);
    module wheel() {
        along_axle(outside,face-outside) circle(d=maximum?wheel_diameter_max:wheel_diameter);
    }
    if(which=="R") translate([board_width,0,0]) mirror([1,0,0]) wheel(); else wheel();
}
module stator_reference(which,maximum=false) {
    projection = maximum?stator_projection_max:stator_projection;
    module stem() {
        along_axle(stator_face_x-projection,projection)
            keyed_profile(maximum?stator_max_diameter:stator_nominal_diameter,
                          maximum?stator_max_flat_radius:stator_nominal_flat_radius);
    }
    if(which=="R") translate([board_width,0,0]) mirror([1,0,0]) stem(); else stem();
}
module assembly() {
    color([.7,.74,.78,.45]) for(name=["2pybot_base_F","2pybot_base_B","2pybot_shell","2pybot_face","beam_D"])
        import(str("../../../../models/",name,".stl"));
    for(which=["L","R"]) {
        color([.05,.70,.75]) mount_assembly(which);
        // Conservative cylindrical reference, not a manufacturer 3D model.
        color([.18,.20,.23]) wheel_envelope(which);
        color([.9,.55,.15]) stator_reference(which);
    }
}
module parameters() {
    echo([ ["board_width",board_width], ["anchor_center_x",motor_anchor_x], ["anchor_center_y",motor_anchor_y],
           ["strut_pitch_x",strut_pitch_x], ["strut_pitch_y",strut_pitch_y], ["axle_drop",axle_drop],
           ["frame_size",frame_size], ["frame_depth",frame_depth], ["frame_rail",frame_rail],
           ["tray_ledge_inner_x",tray_ledge_inner_x], ["tray_ledge_lower_z",tray_ledge_lower_z],
           ["tray_ledge_clearance",tray_ledge_clearance],
           ["stator_face_x",stator_face_x], ["socket_lip_x",socket_lip_x], ["socket_depth",socket_depth],
           ["face_plate_thickness",face_plate_thickness], ["socket_diameter",socket_diameter],
           ["socket_flat_radius",socket_flat_radius], ["socket_outer_diameter",socket_outer_diameter],
           ["bolt_circle",motor_bolt_circle], ["bolt_clearance",motor_bolt_clearance],
           ["thread_depth",motor_thread_depth], ["wheel_diameter",wheel_diameter],
           ["wheel_diameter_min",wheel_diameter_min], ["wheel_diameter_max",wheel_diameter_max],
           ["overall_width",overall_width], ["overall_width_max",overall_width_max],
           ["stator_projection",stator_projection], ["stator_projection_min",stator_projection_min],
           ["socket_diameter_clearance",socket_diameter_clearance], ["socket_flat_clearance",socket_flat_clearance],
           ["motor_clock_deg",motor_clock_deg], ["cable_direction",cable_direction] ]);
}

if(part=="mount") {
    assert(side=="L" || side=="R");
    if(layout=="assembly") mount_assembly(side); else printable_mount(side);
}
else if(part=="chassis_gauge") chassis_gauge();
else if(part=="motor_gauge") motor_gauge();
else if(part=="assembly") assembly();
else if(part=="envelope") wheel_envelope(side,maximum_envelope);
else if(part=="stator") stator_reference(side,maximum_envelope);
else if(part=="parameters") parameters();
else assert(false,"Unknown part selector");
