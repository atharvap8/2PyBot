// Logo B (original option B). width = printed ink width in mm.
module logo_b_2d(width=50){ s=width/45.2; scale([s,s]) import("logo_B.svg",center=true); }
module logo_b(width=50,h=0.8){ translate([0,0,-0.01]) linear_extrude(h+0.01) logo_b_2d(width); }
