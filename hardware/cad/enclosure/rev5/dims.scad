// 2PyBot shared dimensions. ONE frame for every part:
// origin = bottom-deck FRONT-LEFT corner, z=0 = bottom-deck SOLDER (under) face.
// +X = left->right (viewed from front), +Y = front->back, +Z up.
// [ROUGH] = placeholder, fine-tune later.
$fn = 48;
PCB_W=150; PCB_D=90; PCB_T=1.6;
GAP_BOT=60; GAP_CAM=40;                       // bottom->mid, mid->top (clear gap)
Z_MID0=PCB_T+GAP_BOT; Z_MID1=Z_MID0+PCB_T;
Z_TOP0=Z_MID1+GAP_CAM; Z_TOP1=Z_TOP0+PCB_T;
HOLE_IN=3.5; HOLE_D=3.2;                      // corner M3 [ROUGH photo est.]
CORNERS=[[HOLE_IN,HOLE_IN],[PCB_W-HOLE_IN,HOLE_IN],[HOLE_IN,PCB_D-HOLE_IN],[PCB_W-HOLE_IN,PCB_D-HOLE_IN]];
// fits
FIT=0.3;          // locating / sliding fit per side
PILOT=2.5;        // M3 self-tap pilot
CLR=3.4;          // M3 clearance
// USB-C (Radxa power) on bottom deck, LEFT (X=0) edge — LOCKED
USB_Y0=PCB_D-54; USB_Y1=PCB_D-45; USB_Z=4.5;  // centre height above solder face
PLUG_W=12.5; PLUG_H=10;                        // cable housing
USB_OW=PLUG_W+1.0; USB_OH=PLUG_H+1.0;          // opening
USB_YC=(USB_Y0+USB_Y1)/2;
// motor standoffs (brass M3 hex) — SQUARE pattern, 4 per motor [ROUGH]
SO_C=[23,45]; SO_S=30;                          // centre (left motor), side
SO_X=[SO_C[0]-SO_S/2,SO_C[0]+SO_S/2]; SO_Y=[SO_C[1]-SO_S/2,SO_C[1]+SO_S/2]; SO_LEN=40;
SQ=44;                                          // big square opening in tray floor per motor
SQ_X0=SO_C[0]-SQ/2; SQ_Y0=SO_C[1]-SQ/2;
MTAB_X=56; MTAB_Y=[33,57];                      // cover screw tabs (left), self-tap up into tray
function mx(x,s)= s=="L"? x : PCB_W-x;         // mirror left->right
// base tray
FLOOR=2; RECESS=7; TWALL=3; TWALL_UP=6;        // wall rises 6 above board top
LEDGE=2.5;
TRAY_Z0=-(RECESS+FLOOR); TRAY_Z1=PCB_T+TWALL_UP;
TX0=-FIT-TWALL; TX1=PCB_W+FIT+TWALL; TY0=-FIT-TWALL; TY1=PCB_D+FIT+TWALL;
BOSS_Z=-4;                                     // shell->tray horizontal screws
// [x,y,normal] side boss points on tray outer walls
BOSSES=[[30,TY0,"F"],[PCB_W/2,TY1,"B"],[TX0,80,"L"],[TX1,80,"R"]];
// shell
SW=2.8; SFIT=0.3; SHROUD=24; LID_T=2.8;
SX0=TX0-SFIT; SX1=TX1+SFIT; SY0=TY0-SFIT; SY1=TY1+SFIT;
SZ0=TRAY_Z0; LID_Z0=Z_TOP1+SHROUD; SZ1=LID_Z0+LID_T;
// motor (NEMA17) [ROUGH]
NEMA=42.3; MW=2.4; LIFT=3;
FLANGE_Z=-SO_LEN;                               // bracket flange top
M_BOT_Z=FLANGE_Z-3-44;                          // lowest point (encoder board 44 sq)
M_X0=-3; M_X1=48;                               // bracket plate (wheel side) .. encoder pins
M_Y0=SO_C[1]-29; M_Y1=SO_C[1]+29;               // incl. side JST connector
ID_FONT="DejaVu Sans:style=Bold";
module id_text(t,s=5){ linear_extrude(0.61) text(t,size=s,font=ID_FONT,halign="center",valign="center"); }
