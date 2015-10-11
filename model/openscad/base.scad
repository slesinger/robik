//Base 35x37cm

difference() {
linear_extrude(height = 0.006, center = true, convexity = 10, twist = 0, slices = 2, scale = 1.0)
	polygon(points=[ [0.125,0],
	[0.10,-0.175], [0.0,-0.175], [-0.04,-0.176], [-0.14,-0.160], [-0.21,-0.125],
[-0.24,0.0], 
	[-0.21,0.125], [-0.14,0.160], [-0.04, 0.176], [0.0,0.175], [0.10,0.175] ]);

translate([0,0.125+0.11/2,0]) {
	cube(size=0.11, center=true);
}
translate([0,-(0.125+0.11/2),0]) {
	cube(size=0.11, center=true);
}
} //end difference

//mudgear left
translate([0, 0.15,-0.01])
rotate([90,0,0]) {
	difference() {
		cylinder(h=0.05, r=0.11/2, center=true, $fn=30);
		translate([0,0,-0.005])
			cylinder(h=0.05, r=0.11/2-0.001, center=true, $fn=30);
		translate([0,-0.047,-0.029])
			cube(0.11, center=true);
	}
}

//mudgear right
translate([0, -0.15,-0.01])
rotate([-90,180,0]) {
	difference() {
		cylinder(h=0.05, r=0.11/2, center=true, $fn=30);
		translate([0,0,-0.005])
			cylinder(h=0.05, r=0.11/2-0.001, center=true, $fn=30);
		translate([0,-0.047,-0.029])
			cube(0.11, center=true);
	}
}

//center of rotation
cube(size=[0.01, 0.005,0.01], center=true);
%cube([1,0.01,0.01], center=true);
