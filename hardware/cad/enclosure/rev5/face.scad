// 2PyBot camera face C-plate (the ONLY part with the logo).
include <dims.scad>
use <logo.scad>
LENS_CX=52; LENS_H=17; LENS_W=26; LENS_HT=18;   // rectangular lens cut [ROUGH]
TOF_CX=118.5; TOF_D=9; TOF_H=12;
T1=2.4; TAB=10;
CN=[[HOLE_IN,HOLE_IN],[PCB_W-HOLE_IN,HOLE_IN]];
difference(){
  union(){
    translate([0,-T1,Z_MID1]) cube([PCB_W,T1,Z_TOP1+T1-Z_MID1]);
    for(x=[0,PCB_W-TAB]){
      translate([x,-T1,Z_TOP1]) cube([TAB,TAB+T1,T1]);
      translate([x,-T1,Z_MID1]) cube([TAB,TAB+T1,T1]);
    }
  }
  translate([LENS_CX-LENS_W/2,-T1-1,Z_MID1+LENS_H-LENS_HT/2]) cube([LENS_W,T1+2,LENS_HT]);
  translate([TOF_CX,1,Z_MID1+TOF_H]) rotate([90,0,0]) cylinder(d=TOF_D,h=T1+4);
  for(c=CN){
    translate([c[0],c[1],Z_TOP1-1]) cylinder(d=CLR,h=T1+2);
    hull(){ translate([c[0],c[1],Z_MID1-1]) cylinder(d=CLR+0.4,h=T1+2);
            translate([c[0],TAB+2,Z_MID1-1]) cylinder(d=CLR+0.4,h=T1+2); }
  }
  translate([PCB_W/2,-0.6,Z_MID1+30]) rotate([90,0,180]) id_text("FACE",5);
}
translate([122,-T1+0.01,Z_MID1+31]) rotate([90,0,0]) logo_b(34,0.8);
