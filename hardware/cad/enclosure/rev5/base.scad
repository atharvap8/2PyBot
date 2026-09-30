// 2PyBot base / solder tray. PCB drops in (0.3/side), rests on ledge + 4 corner posts,
// M3 self-tap through the PCB corner holes into the posts.
include <dims.scad>
module tray(){
  difference(){
    union(){
      difference(){
        translate([TX0,TY0,TRAY_Z0]) cube([TX1-TX0,TY1-TY0,TRAY_Z1-TRAY_Z0]);
        translate([-FIT,-FIT,0]) cube([PCB_W+2*FIT,PCB_D+2*FIT,50]);                     // board well
        translate([-FIT+LEDGE,-FIT+LEDGE,TRAY_Z0+FLOOR]) cube([PCB_W+2*FIT-2*LEDGE,PCB_D+2*FIT-2*LEDGE,RECESS+0.01]);
      }
      for(c=CORNERS) translate([c[0],c[1],TRAY_Z0]) cylinder(d=8,h=-TRAY_Z0);            // corner posts
      for(b=BOSSES) boss_pad(b);                                                          // inward screw pads (below board)
      for(s=["L","R"]) for(y=MTAB_Y) translate([mx(MTAB_X,s),y,TRAY_Z0]) cylinder(d=8,h=FLOOR+4);
    }
    for(c=CORNERS) translate([c[0],c[1],TRAY_Z0+FLOOR]) cylinder(d=PILOT,h=20);
    for(b=BOSSES) boss_hole(b);
    for(s=["L","R"]){
      translate([s=="L"?SQ_X0:PCB_W-SQ_X0-SQ,SQ_Y0,TRAY_Z0-1]) cube([SQ,SQ,FLOOR+1.01]);   // big square
      for(y=MTAB_Y) translate([mx(MTAB_X,s),y,TRAY_Z0-1]) cylinder(d=PILOT,h=FLOOR+5);
    }
    translate([PCB_W/2,PCB_D/2,TRAY_Z0+FLOOR-0.6]) id_text("BASE",7);
    // USB-C: open-top U-notch in the LEFT wall so the board + receptacle drop in
    translate([TX0-1,USB_YC-USB_OW/2,USB_Z-USB_OH/2]) cube([TWALL+FIT+1,USB_OW,50]);
  }
}
module boss_pad(b){
  if(b[2]=="F") translate([b[0]-6,-FIT,TRAY_Z0]) cube([12,6,-TRAY_Z0-0.5]);
  if(b[2]=="B") translate([b[0]-6,PCB_D+FIT-6,TRAY_Z0]) cube([12,6,-TRAY_Z0-0.5]);
  if(b[2]=="L") translate([-FIT,b[1]-6,TRAY_Z0]) cube([6,12,-TRAY_Z0-0.5]);
  if(b[2]=="R") translate([PCB_W+FIT-6,b[1]-6,TRAY_Z0]) cube([6,12,-TRAY_Z0-0.5]);
}
module boss_hole(b){
  r = b[2]=="F"?[-90,0,0]: b[2]=="B"?[90,0,0]: b[2]=="L"?[0,90,0]:[0,-90,0];
  translate([b[0],b[1],BOSS_Z]) rotate(r) translate([0,0,-1]) cylinder(d=PILOT,h=10);
}
tray();
