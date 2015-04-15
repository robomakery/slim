

m3_nut_diameter= 7;
m3_diameter=4.4;

module block(){


difference(){
union(){
difference(){
// cube([22,22,24]); 


//translate([11,11,3]) cylinder(72,7.55,7.55,$fn=100);
//translate([11,11,-0.1]) cylinder(72,6,6,$fn=100);

}


translate([-9,16,0]) cube ([40,8,24]);

translate([11,11,-0.1]) cylinder(24,12,12,$fn=100);

 

}


translate([10,-2,-0.1]) cube([2,14,26]); 

translate([11,11,3]) cylinder(72,7.55,7.55,$fn=100);
translate([11,11,-0.1]) cylinder(72,6,6,$fn=100);

// UNCOMMENT FOR NO END STOP
translate([11,11,-0.1]) cylinder(72,7.62,7.62,$fn=100);

translate(v = [-4.5,14, 5]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex 
translate(v = [-4.5, 14, 5]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole

translate(v = [26.5,14, 5]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex 
translate(v = [26.5, 14, 5]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole

translate(v = [-4.5,14, 19]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex 
translate(v = [-4.5, 14,19]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole

translate(v = [26.5,14,19]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex 
translate(v = [26.5, 14, 19]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole


//translate(v = [-4, -8, 2]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex
//translate(v = [-4, -8, 3.75]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole

}
}

translate([10,0,0])
block();

//translate(v = [-4,20, 5]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex 
//translate(v = [-4, 20, 5]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole
//translate(v = [-4, -8, 2]) rotate(a=[270,0,0]) cylinder(h = 9, r=m3_nut_diameter/2, $fn=6, center=true); // M3 hex
//translate(v = [-4, -8, 3.75]) rotate(a=[270,0,0]) cylinder(h = 100, r=m3_diameter/2, $fn=9, center=true); // M3 hole