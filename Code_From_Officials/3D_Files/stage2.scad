// ***************************************************************
// * Southeastcon 2107
// *
// * Designs of the 3D printed pieces needed for Stage 2 of 
// *   the hardware robotics competition. This file contains 
// *   the lightsaber and the electromagnet coil spool.
// *
// * For more details about the competition go to
// *    http://sites.ieee.org/southeastcon2017
// *
// * Designed by Rodney Radford
// ***************************************************************


// This value controls the accuracy of the cylinders and spheres
// For fast drawing (while making changes), choose a low value (20)
// For more accurate/smoother surfaces, choose a higher value (200)
// The higher value increases computation/drawing time
$fn=20;

// ***************************************************************
// Drawing parameters
// ***************************************************************
fudge = 0.01;

hilt_length = 90;
hilt_radius = 15;
tenon_radius = 9;
tenon_length = 10;

saber_length = 85;
saber_major_radius = 12;
saber_minor_radius = 8;
saber_wall_thickness = 1.5;
inside_radius = 7;

neopixel_zoffset = 15;
neopixel_length  = 50;
neopixel_width   = 6;

bracket_thickness = 6;
screw_radius = 3;

flange_radius = 15;
flange_thickness = 2;
spool_radius = 25.4/4;
spool_width = 25;

hole_radius = 4;
inner_radius = (spool_radius + hole_radius) / 2;
clearance = 0.1;
wire_radius = 1.5;

// ***************************************************************
//
// ***************************************************************

// Look at the source for the 'draw' fun1ction to see what values can 
//   be passed in to it to control rendering parts of the model at a 
//   time for review and/or printing

draw("everything");            // see everything
//draw("bracket+hilt");        // show the hilt and bracket (black PLA)
//draw("blade");               // lightsaber blade (PLA clear)
//draw("innersupport");        // inner support (PLA clear)
//draw("spool_outer");         // 1/2 of spool (PLA any color)
//draw("spool_inner");         // 1/2 of spool (PLA any color)

module draw(what) {

   // Draw EVERYTHING - depending on your 3D printer, this mode is not recommended
   //    for a couple of reasons. First, it does not allow the hilt and blade to be
   //    printed in different colors (unless you pause the print and switch). Second,
   //    and most important, is the printer may try to bridge the two parts together,
   //    either intentionally or uninentionally with lots of small strands between the
   //    saber and the spool.
   // But this mode is cool to see all the parts at one time... ;-)
   if (what == "everything") {
      bracket();
      translate([0,0,bracket_thickness]) {
        lightsaber_hilt();
        translate([0,0,hilt_length]) {
           lightsaber_blade(window="true");
           lightsaber_innersupport();
        }
     }

      translate([0,hilt_radius*5,0]) {
         spool_inner();
         translate([0,0,spool_width+flange_thickness]) mirror(v=[0,0,1])
            spool_outer();
      }

   // Just render the hilt - great for 3D printing just this piece out
   // The 'official' hilt will be printed with PLA black, and then painted
   //    silver and gold colors. 
   } else if (what == "hilt") {
        lightsaber_hilt();   

   // Just render the blade - also great for 3D printing just this piece out
   // The official blade will be printed in PLA white.
   } else if (what == "blade") {
      lightsaber_blade();
  //    lightsaber_blade(cutaway="true");
  //    lightsaber_blade(window="true", cutaway="false");

   // Just the bracket
   } else if (what == "bracket") {
      bracket(cutaway="true");
   
   } else if (what == "innersupport") {
      lightsaber_innersupport();

   // bracket and hilt so it can be attached to the stage 2 wall
   } else if (what == "bracket+hilt") {
      bracket();
      translate([0,0,bracket_thickness])
        lightsaber_hilt();

   // Hilt and blade - not really useful for the contest as they need to
   //   be mounted to something, but a) cool to see it together, and b) it
   //   would be fun to see this used as the offensive weapon on the robot
   //   (is anyone up to that challenge?)
   } else if (what == "hilt+blade") {
        lightsaber_hilt();
        translate([0,0,hilt_length])
           lightsaber_blade();

   // Render it all together for visual inspection
   } else if (what == "saber") {
      bracket();
      translate([0,0,bracket_thickness]) {
        lightsaber_hilt();
        translate([0,0,hilt_length])
           lightsaber_blade(cutaway="true");
      }

   // Render just the inner spool for 3D printing (PLA, color not important)
   } else if (what == "spool_inner") {
      spool_inner();

   // Render just the outer spool for 3D printing (PLA, color not important)
   } else if (what == "spool_outer") {
      spool_outer();

   // Render both halves of the spool to verify fit
   } else if (what == "spool") {
      spool_inner();
      translate([0,0,spool_width+flange_thickness]) mirror(v=[0,0,1])
         spool_outer();
   }
}


// ***************************************************************
// 
// ***************************************************************

module lightsaber_hilt() {

difference() {

   union() {

      // major cylinder of the hilt
      color("black") cylinder(r=hilt_radius, h=hilt_length);
      translate([0,0,5])
         color("silver") cylinder(r=hilt_radius+fudge, h=55);

      // add the blade tenon on top
      color("red") translate([0,0,hilt_length])
         cylinder(r=tenon_radius, h=tenon_length);
   }

   // hollow out the center for wires
   translate([0,0,-fudge])
      cylinder(r=inside_radius, h=hilt_length+tenon_length+2*fudge);

   // remove the cylindrical grooves in the lower part of the hilt
   translate([0,0,12])
      for (i = [0:12]) {
         rotate(i*30)
         translate([0,hilt_radius-0.5,0])
         color("black") cylinder(r=3,h=40);
      }
}

// draw the three bands around the hilt
color("gold") for (z = [5,60,86]) {
 translate([0, 0, z]) 
    rotate_extrude(convexity = 10)
       translate([hilt_radius, 0, 0])
       circle(r = 3);
}

// add the square grooves at the upper part of the hilt
translate([0,0,(60+86)/2]) {
   for (i = [0:12]) {
      rotate(i*30)
      translate([0,hilt_radius,0])
        color("silver") cube([4,4,15], center=true);
      }
   }

// draw the button in the center of the upper part of the hilt
translate([0,0,(60+86)/2]) {
   rotate(a=90,v=[0,1,0]) {
      translate([0,0,inside_radius+2])
      color("silver") cylinder(r=5, h=8);
      translate([0,0,inside_radius+2+8])
      color("gold") sphere(r=4);
      }
   }
}


// ***************************************************************
// 
// ***************************************************************

module lightsaber_blade(window="false", cutaway="false") {

   color("white") difference() {
      union() {

         // straight part at the bottom half
         cylinder(r=saber_major_radius, h=saber_length/2);

         // followed by tapered top half
         translate([0,0,saber_length/2])
            cylinder(r1=saber_major_radius, r2=saber_minor_radius, h=saber_length/2);

         // then add the rounded top
         translate([0,0,saber_length]) 
            sphere(r=saber_minor_radius);   
      }

      // add straight walls to fit around the hilt tenon
      translate([0,0,-fudge]) 
         cylinder(r=tenon_radius, h=tenon_length+2*fudge);

      // hollow out the  bottom half of the saber
      translate([0,0,tenon_length]) 
         cylinder(r=tenon_radius, h=saber_length/2 - tenon_length);

      // hollow out the top half of the saber
      translate([0,0,saber_length/2 - fudge])
         cylinder(r1=saber_major_radius-saber_wall_thickness, 
                  r2=saber_minor_radius-saber_wall_thickness, 
                  h=saber_length/2);

      // the "window" option allows printing with non-clear PVA and
      //    opens up a small window in the blade so you can see the LEDs
      if (window == "true") {
          translate([0,-neopixel_width*.55,neopixel_zoffset])
             cube([saber_major_radius+fudge*2, 
                   neopixel_width*1.1, 
                   neopixel_length]);
      }  

      // the cutaway option allows seeing the interior of the blade
      if (cutaway == "true") {
         rotate(-90) translate([0,-saber_major_radius-fudge,-fudge])
            cube([saber_major_radius+  + 2*fudge, 
                  saber_major_radius*2 + 2*fudge, 
                  saber_length*1.5]);
      }
   }
}


module lightsaber_innersupport() {

   color("red") rotate(180) difference() {
      union() {
          cylinder(r=inside_radius-clearance*3, h=tenon_length);
          translate([0,0,tenon_length]) 
             cylinder(r=tenon_radius-1, h=flange_thickness);
          translate([0,-6,tenon_length+flange_thickness])
             cube([flange_thickness*2, 12, neopixel_zoffset+30]);
       }

       // carve out area to solder wires to neopixel
       translate([-fudge,-6-fudge,tenon_length+flange_thickness])
          cube([flange_thickness+fudge,12+2*fudge, 
               neopixel_zoffset+8-tenon_length-flange_thickness]);

      // carve out an area for a ziptie to hold in the vibration sensor
      translate([-fudge,-6-fudge, neopixel_zoffset+24])
         cube([flange_thickness+fudge,12+2*fudge, 4]);
      translate([-fudge,-6-fudge,neopixel_zoffset+24])
         cube([flange_thickness*2+2*fudge,2,4]);
      translate([-fudge,6+fudge-2,neopixel_zoffset+24])
         cube([flange_thickness*2+2*fudge,2,4]);

      translate([neopixel_zoffset+24,0,-10])
         cube([4,-4+fudge,20]);
 

      // carve out channel for wires
      translate([-4,-5,-fudge]) 
          cube([4, 10, tenon_length+flange_thickness+2*fudge]);

       rotate(a=-90, v=[0,1,0]) { // Z,Y,-X

          translate([tenon_length+flange_thickness,0,-10])
             cylinder(r=3, h=14);

          translate([neopixel_zoffset, 0, 0]) {
             translate([12,      2.7,-10]) cylinder(r=1.5, h=20);
             translate([12,     -2.7,-10]) cylinder(r=1.5, h=20);
             translate([12+25.4, 2.7,-10]) cylinder(r=1.5, h=20);
             translate([12+25.4,-2.7,-10]) cylinder(r=1.5, h=20);
          }
       }
   }
}


// ***************************************************************
//
// ***************************************************************

module bracket() {

   color("white") difference() {
      union() {
         // middle 'square' area of the bracket
         translate([-hilt_radius, -2*hilt_radius, 0])
            cube([hilt_radius*2, hilt_radius*4, bracket_thickness]);

         // add the rounded ends to the bracket
         translate([0, -2*hilt_radius, 0])
            cylinder(r=hilt_radius, h=screw_radius*2);
         translate([0, 2*hilt_radius, 0])
            cylinder(r=hilt_radius, h=screw_radius*2);
      }

      // drill out the two mounting holes
      translate([0,-2.2*hilt_radius, -fudge])
         cylinder(r=screw_radius, h=bracket_thickness + 2*fudge);
      translate([0,2.2*hilt_radius, -fudge])
         cylinder(r=screw_radius, h=bracket_thickness + 2*fudge);

      // drill out the inner radius (to route wires out bracket hole)
      translate([0,0,-fudge])
         cylinder(r=inside_radius, h=bracket_thickness + 2*fudge);

      translate([-hilt_radius-fudge,-0.5*bracket_thickness,-fudge])
         cube([hilt_radius+inside_radius, 
               bracket_thickness, 
               bracket_thickness * 0.8]);

     }
}


      // drill out a hole in the bracket to run the wires
//      translate([0,0,0.5*bracket_thickness]) 
//         rotate(a=-90, v=[0,1,0])
//         cylinder(r=bracket_thickness*0.4, h=hilt_radius+2*fudge);

      // remove a channel to allow wires to exit out the back of the hilt
//      translate([-hilt_radius,-0.5*bracket_thickness,-fudge])
//         cube([hilt_radius+inside_radius+2*fudge, 
//               bracket_thickness, 
//               2*bracket_thickness +2*fudge]);



// ***************************************************************
//
// ***************************************************************

module spool_inner() {

   color("blue") difference() {
      union() {

         // draw the the inner spool's flange
         cylinder(r=flange_radius, h=flange_thickness);

         // draw the spool inner cylinder
         cylinder(r=inner_radius-(clearance/2), h=spool_width);
      }

      // drill the hole through the middle of the spool
      translate([0,0,-clearance])
         cylinder(r=hole_radius, h=spool_width+3*flange_thickness);
   }
}


// ***************************************************************
//
// ***************************************************************

module spool_outer() {

   color("blue") difference() {
      union() {

         // draw the outer spool's flange
         cylinder(r=flange_radius, h=flange_thickness);

         // draw the spool outer cylinder (slips over the other half)
         cylinder(r=spool_radius, h=spool_width);
      }

      // drill the hole through the middle of the spool
      translate([0,0,-clearance])
         cylinder(r=hole_radius, h=2*clearance + flange_thickness*2);
      translate([0,0,flange_thickness-clearance])
         cylinder(r=inner_radius+(clearance/2), h=spool_width+2*clearance);

      // drill out the two holes for the wires in the side of the spool
      translate([spool_radius+wire_radius,0,-clearance])
         cylinder(r=wire_radius, h=(flange_thickness+clearance)*2); 
      translate([spool_radius+wire_radius*3.1,-clearance])
         cylinder(r=wire_radius, h=(flange_thickness+clearance)*2); 
   }
}
