// 2PyBot motor cover (L/R). Two sections: upper spacer shroud + lower motor box.
// Wheel side OPEN. Fit: slide OUTBOARD over motor + standoffs, then push UP (LIFT)
// so the spigot seats in the tray's square opening; 2 M3 self-taps up into tray bosses.
// render: openscad -D 'side="L"' -o motor_L.stl motor.scad
include <dims.scad>
side="L";
UX0=SQ_X0+FIT; UX1=SQ_X0+SQ-FIT; UY0=SQ_Y0+FIT; UY1=SQ_Y0+SQ-FIT;   // spigot/shroud outer
UZ1=TRAY_Z0+FLOOR;                                                  // spigot top (in floor)
LZ1=FLANGE_Z+LIFT+0.5;                                              // lower box inner top
LZ0=M_BOT_Z-1-LIFT;                                                 // lower box inner bottom
module body(){
  difference(){
    union(){
      // lower motor box
      translate([M_X0,M_Y0-MW,LZ0-MW]) cube([M_X1-M_X0+MW,M_Y1-M_Y0+2*MW,LZ1-LZ0+2*MW]);
      // upper shroud (3 walls, outboard open)
      translate([UX0,UY0,LZ1]) cube([UX1-UX0,UY1-UY0,UZ1-LZ1]);
      // screw tabs from shroud inboard wall
      for(y=MTAB_Y) hull(){
        translate([MTAB_X,y,TRAY_Z0-3]) cylinder(d=9,h=3);
        translate([UX1-0.01,y-4.5,TRAY_Z0-3]) cube([1,9,3]);
      }
    }
    translate([M_X0-1,M_Y0,LZ0]) cube([M_X1-M_X0+1,M_Y1-M_Y0,LZ1-LZ0]);                // motor cavity
    translate([M_X0-1,UY0+MW,LZ1-1]) cube([UX1-MW-M_X0+1,UY1-UY0-2*MW,UZ1-LZ1+2]);    // shroud bore, open outboard
    for(y=MTAB_Y) translate([MTAB_X,y,TRAY_Z0-5]) cylinder(d=CLR,h=7);
    for(i=[0:3]) translate([M_X1-1,M_Y0+10+i*10,LZ0+10]) cube([MW+2,4,LZ1-LZ0-20]); // inboard vents
  }
}
if(side=="R") difference(){ translate([PCB_W,0,0]) mirror([1,0,0]) body();
  translate([PCB_W-M_X1-0.6,(M_Y0+M_Y1)/2,LZ0+5]) rotate([90,0,90]) id_text("R COVER",5); }
else difference(){ body();
  translate([M_X1+0.6,(M_Y0+M_Y1)/2,LZ0+5]) rotate([90,0,-90]) id_text("L COVER",5); }
