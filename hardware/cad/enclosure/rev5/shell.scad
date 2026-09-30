// 2PyBot main shell rev5: rounded vertical edges, chamfered lid, no fins/ribs/logos.
// Slides down onto the tray (0.3/side); 4 horizontal M3 self-taps into tray pads,
// 2 lid screws (M3 + nut) at the top-deck back corners. NeoPixel ring in a ridged channel.
include <dims.scad>
RING_OD=68; RING_ID=54; RING_CX=44; RING_CY=45; LED_R=(RING_OD+RING_ID)/4;
FAN_D=30; FAN_CX=105; FAN_CY=45;
TORCH_X0=59; TORCH_X1=140; TORCH_Z1=Z_MID0-1;
CAM_X0=6; CAM_X1=144; CAM_Z0=Z_MID1+3; CAM_Z1=Z_TOP1-2;
RAD_X0=78; RAD_X1=147; RAD_Z0=Z_TOP1+1; RAD_Z1=LID_Z0-2;
OX0=SX0-SW; OX1=SX1+SW; OY0=SY0-SW; OY1=SY1+SW;
CH=6; RI=1.0; RO=RI+SW;
module rr(x0,y0,x1,y1,r){ translate([x0+r,y0+r]) offset(r=r) square([x1-x0-2*r,y1-y0-2*r]); }
module slab(x0,y0,x1,y1,r,z0,z1){ translate([0,0,z0]) linear_extrude(z1-z0) rr(x0,y0,x1,y1,r); }
module outer(){ hull(){ slab(OX0,OY0,OX1,OY1,RO,SZ0,SZ1-CH); slab(OX0+CH,OY0+CH,OX1-CH,OY1-CH,RO,SZ1-0.01,SZ1); } }
module inner(){ hull(){ slab(SX0,SY0,SX1,SY1,RI,SZ0-1,LID_Z0-CH); slab(SX0+CH,SY0+CH,SX1-CH,SY1-CH,RI,LID_Z0-0.01,LID_Z0); } }
// ring channel: outer + inner ridge, wire gap, 3 snap nubs
RH=4.5;
module ring_channel(){
  z=LID_Z0-RH;
  translate([RING_CX,RING_CY,z]) difference(){
    union(){
      difference(){ cylinder(r=RING_OD/2+0.3+1.6,h=RH+0.01,$fn=128); translate([0,0,-1]) cylinder(r=RING_OD/2+0.3,h=RH+2,$fn=128); }
      difference(){ cylinder(r=RING_ID/2-0.3,h=RH+0.01,$fn=128); translate([0,0,-1]) cylinder(r=RING_ID/2-0.3-1.6,h=RH+2,$fn=128); }
      for(a=[60,180,300]) rotate([0,0,a]) translate([RING_OD/2+0.3-0.5,-2,0.4]) cube([0.6,4,0.8]);
    }
    rotate([0,0,0]) translate([RING_OD/2-2,-4,-1]) cube([6,8,RH+2]);   // wire exit gap (+X side)
  }
}
difference(){
  union(){ difference(){ outer(); inner(); } ring_channel();
    for(c=[CORNERS[2],CORNERS[3]]) translate([c[0],c[1],Z_TOP1+0.2]) cylinder(d=8,h=LID_Z0-Z_TOP1);
  }
  translate([CAM_X0,OY0-1,CAM_Z0]) cube([CAM_X1-CAM_X0,SW+2,CAM_Z1-CAM_Z0]);
  translate([TORCH_X0,OY0-1,SZ0-1]) cube([TORCH_X1-TORCH_X0,SW+2,TORCH_Z1-SZ0+1]);
  translate([RAD_X0,OY0-1,RAD_Z0]) cube([RAD_X1-RAD_X0,SW+CH+2,RAD_Z1-RAD_Z0]);
  translate([OX0-1,USB_YC-USB_OW/2,SZ0-1]) cube([SW+2,USB_OW,USB_Z+USB_OH/2-SZ0+1]);
  translate([OX0-1,USB_YC,USB_Z+USB_OH/2]) rotate([0,90,0]) cylinder(d=USB_OW,h=SW+2,$fn=64);
  translate([OX0-1,15,6.5]) cube([SW+2,10,5]);                                       // ESP cable
  for(k=[0:15]) translate([RING_CX+LED_R*cos(k*22.5+11.25),RING_CY+LED_R*sin(k*22.5+11.25),LID_Z0-1]) cylinder(d=5,h=LID_T+2);
  translate([FAN_CX,FAN_CY,LID_Z0-1]) cylinder(d=FAN_D,h=LID_T+2);
  for(c=[CORNERS[2],CORNERS[3]]){
    translate([c[0],c[1],Z_TOP1-1]) cylinder(d=CLR,h=SZ1-Z_TOP1+2);
    translate([c[0],c[1],SZ1-3]) cylinder(d=6.4,h=4);
  }
  for(b=BOSSES){
    r = b[2]=="F"?[90,0,0]: b[2]=="B"?[-90,0,0]: b[2]=="L"?[0,-90,0]:[0,90,0];
    translate([b[0],b[1],BOSS_Z]) rotate(r) translate([0,0,-SW-3]) cylinder(d=CLR,h=SW+6);
  }
  // single accent groove
  difference(){ slab(OX0-1,OY0-1,OX1+1,OY1+1,RO+1,50,51.2); slab(OX0+0.8,OY0+0.8,OX1-0.8,OY1-0.8,RO-0.8,49,52.2); }
  translate([PCB_W/2,SY1+0.6,60]) rotate([90,0,0]) id_text("SHELL",7);
}
