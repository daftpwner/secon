// ********************************************
// 2017 IEEE SoutheastCon Hardware Competition
// Episode MMXVII layout
// ********************************************

// Conversion factors
inch  = 25.4; // mm to inch conversion
fudge = 0.1;  // mm for z-fighting

// Standard lumber dimensions
by1 = 0.75*inch;
by2 = 1.5*inch;
by4 = 3.5*inch;
halfInchPly = 15/32*inch; // nominal 0.5"

// Arena drawFloor dimensions
arenaX = 8*12*inch;
arenaY = 4*12*inch;
arenaZ = halfInchPly;

// Step dimensions
step1X = 12*inch;
step2X = 12*inch;
step3X = 12*inch;
step4X = 12*inch;
step4Y = 12*inch;
stepOverlap = 2*inch;

// Starting square dimensions
startX = 15*inch;
startY = 15*inch;

// stage 1 dimensions
stage1X = 6*inch;
stage1Z = 6*inch;

// stage 2 dimensions
stage2Y = 6*inch;
stage2Z = 3*inch;
saberZ = 3*inch;
saberD = 0.5*inch;
stage2Hole = by4/2;

// Stage 3 dimensions
stage3X = 6*inch;
stage3Z = 6*inch;
knobThickness = 12.5; // mm
knobD1 = 15; // mm
knobD2 = 14.5; // mm

// Stage 4 dimensions
finalHole = 6*inch+by1+by1;
finalZ = arenaZ+by4+12*inch;
finalHoleHeight=arenaZ+7*halfInchPly+by4; // 3.5" above step4

// The lowest area of the floor area 
floorX = arenaX-by2-step3X-step2X-step1X-by2;

bolt_diameter = 3/8 * inch;

// Arena colors
wall_color = "white";
floor_color = "black";
stage_wall_color = "black";
stage_trim_color = "white";
stripe_color = "white";
stripe_width = 1*inch;
bolt_color = "silver";
pcb_color = "Chocolate";
stage3_knob_color = "grey";

module drawFloor()
{
    color(floor_color) 
        cube([arenaX, arenaY, arenaZ]);
}

module drawBoltHole()
{
    union() { 
        cylinder(r=0.75*inch/2, h=0.5*inch+fudge);
        cylinder(r=bolt_diameter/2, 
            h=by2+fudge+fudge);
    }
}

module drawBolt()
{
    // bolt heads - 3/8" bolt uses 9/16" wrench
    headR = sqrt(9/32*9/32+9/32*9/32) * inch;

    // bolt head
    color(bolt_color)
        cylinder(r=headR, h=0.25*inch, $fn=6);
}

module drawWalls()
{
    color(wall_color) 
    difference() {
        union() {
            translate([by2, 0, arenaZ]) 
            cube([arenaX-by2, by2, by4]);
            translate([0, arenaY-by2, arenaZ]) 
            cube([arenaX-by2, by2, by4]);
            translate([0, 0, arenaZ]) 
            cube([by2, arenaY-by2, by4]);
            translate([arenaX-by2, by2, arenaZ]) 
            cube([by2, arenaY-by2, by4]);
        }
        
        // stage 1 bolt
        translate(
            [arenaX-by2-floorX/2, 
            arenaY-by2-fudge, 
            arenaZ+by4/2]) 
        rotate([-90, 0, 0])
        drawBoltHole()
        drawBolt();
        
        // stage 2 bolt
        translate([arenaX-by2-fudge, 
            arenaY/2, 
            arenaZ+stage2Hole]) 
        rotate([0, 90, 0])
        drawBoltHole()
        drawBolt();
        
        // stage 3 bolt
        translate([arenaX-by2-floorX/2, 
            by2+fudge, 
            arenaZ+by4/2]) 
        rotate([90, 0, 0])
        drawBoltHole()
        drawBolt();
    }
}

module drawStep1()
{
    color(floor_color) 
        translate([by2+step3X+step2X-stepOverlap, 
            by2, 
            arenaZ])
        cube([step1X+stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);

    color(floor_color) 
        translate([by2, 
            by2, 
            arenaZ])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2+step2X-stepOverlap, 
            by2, 
            arenaZ])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    // stripe
    color(stripe_color) 
        translate([
            by2+step3X+step2X+step1X-stripe_width, 
            by2, 
            arenaZ+halfInchPly])
        cube([stripe_width, 
            arenaY-by2-by2,
            fudge]);
}

module drawStep2()
{
    color(floor_color) 
        translate([by2+step3X-stepOverlap, 
            by2, 
            arenaZ+2*halfInchPly])
        cube([step2X+stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2, 
            by2, 
            arenaZ+halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2+step3X-stepOverlap, 
            by2, 
            arenaZ+halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
            
    color(floor_color) 
        translate([by2+step3X+step2X-stepOverlap, 
            by2, 
            arenaZ+halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
            
    color(floor_color) 
        translate([by2, 
            by2, 
            arenaZ+2*halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    // stripe
    color(stripe_color) 
        translate([by2+step3X+step2X-stripe_width, 
            by2, 
            arenaZ+3*halfInchPly])
        cube([stripe_width, 
            arenaY-by2-by2, 
            fudge]);
    
}

module drawStep3()
{
    color(floor_color) 
        translate([by2, 
            by2, 
            arenaZ+4*halfInchPly])
        cube([step3X, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2, 
            by2, 
            arenaZ+3*halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2+step3X-stepOverlap, 
            by2, 
            arenaZ+3*halfInchPly])
        cube([stepOverlap, 
            arenaY-by2-by2, 
            halfInchPly]);
    
    // stripe
    color(stripe_color) 
        translate([by2+step3X-stripe_width, 
            by2, 
            arenaZ+5*halfInchPly])
        cube([stripe_width, 
            arenaY-by2-by2, 
            fudge]);
}

module drawStep4()
{
    color(floor_color) 
        translate([by2, 
            arenaY/2-step4Y/2, 
            arenaZ+6*halfInchPly])
        cube([step4X, step4Y, halfInchPly]);
    
    color(floor_color) 
        translate([by2, 
            arenaY/2-step4Y/2, 
            arenaZ+5*halfInchPly])
        cube([stepOverlap, 
            step4Y-stepOverlap, 
            halfInchPly]);
    
    color(floor_color) 
        translate([by2+stepOverlap, 
            arenaY/2-step4Y/2, 
            arenaZ+5*halfInchPly])
        cube([step4X-stepOverlap, 
            stepOverlap, 
            halfInchPly]);
            
    color(floor_color) 
        translate([by2+step4X-stepOverlap, 
            arenaY/2-step4Y/2+stepOverlap, 
            arenaZ+5*halfInchPly])
        cube([stepOverlap, 
            step4Y-stepOverlap, 
            halfInchPly]);
            
    color(floor_color) 
        translate([by2, 
            arenaY/2+step4Y/2-stepOverlap, 
            arenaZ+5*halfInchPly])
        cube([step4X-stepOverlap, 
            stepOverlap, 
            halfInchPly]);
    
    // stripe
    color(stripe_color) 
        translate([by2+step4X-stripe_width, 
            arenaY/2-step4Y/2, 
            arenaZ+7*halfInchPly])
        cube([stripe_width, 
            step4Y, 
            fudge]);
        
    color(stripe_color) 
        translate([by2, 
            arenaY/2-step4Y/2, 
            arenaZ+7*halfInchPly])
        cube([step4X-stripe_width, 
            stripe_width, 
            fudge]);
            
    color(stripe_color) 
        translate([by2, 
            arenaY/2+step4Y/2-stripe_width, 
            arenaZ+7*halfInchPly])
        cube([step4X-stripe_width, 
            stripe_width, 
            fudge]);
}

module drawStartSquare()
{
    color(stripe_color) 
        translate([arenaX-by2-floorX/2-startX/2, 
            arenaY/2-startY/2, 
            arenaZ])
        cube([startX, startY, fudge]);
    
    // Stripe heading half way to stage 1
    color(stripe_color) 
        translate([
            arenaX-by2-floorX/2-stripe_width/2,
            arenaY/2+startY/2, 
            arenaZ])
        cube([stripe_width, 
            (arenaY/2-startY/2-by2)/2, 
            fudge]);
}

module drawStage1()
{
    difference()
    {
        union()
        {
            // wall
            color(stage_wall_color)
                cube([stage1X, 
                    halfInchPly, 
                    arenaZ+by4+stage1Z]);
            
            // frame
            color(stage_trim_color) {
                translate([-by1, 
                    0, 
                    arenaZ+by4])
                cube([by1, by2, stage1Z+by1]);
                translate([0, 
                    0, 
                    arenaZ+by4+stage1Z])
                cube([stage1X, by2, by1]);
                translate([stage1X, 
                    0, 
                    arenaZ+by4])
                cube([by1, by2, stage1Z+by1]);
            }
        }
            
        // holes for pads
        translate([stage1X/2, 
            halfInchPly+fudge, 
            arenaZ+by4+stage1Z/2])
        {
            // center
            rotate([90, 0, 0]) 
            cylinder(r=0.5*inch/2, 
                h=halfInchPly+fudge+fudge);

            // outer pads
            for (i=[0:4]) {
                rotate([0, i*72, 0])
                translate([0, 0, 1.5*inch])
                rotate([90, 0, 0])
                cylinder(r=0.5*inch/2, 
                    h=halfInchPly+fudge+fudge);
            }
        }
        
        // mounting hole
        translate([stage1X/2, 
            halfInchPly+fudge, 
            arenaZ+by4/2])
        rotate([90, 0, 0])
        cylinder(r=3/8*inch/2, 
            h=halfInchPly+fudge+fudge);
    }
    
    // PCB copper showing through the holes
    color(pcb_color) 
        translate([0, 
            -fudge, 
            arenaZ+by4]) 
        cube([stage1X, fudge, stage1Z]);
}

module drawStage2()
{
    difference() {
        union() {
            // wall
            color(stage_wall_color)
                cube([halfInchPly, 
                stage2Y, 
                arenaZ+by4+stage2Z]);
            
            // frame
            color(stage_trim_color) {
                translate([-by2+halfInchPly, 
                    0, 
                    arenaZ+by4+stage2Z])
                cube([by2, stage2Y, by1]);
                translate([-by2+halfInchPly, 
                    -by1, 
                    arenaZ+by4])
                cube([by2, by1, stage2Z+by1]);
                translate([-by2+halfInchPly, 
                    stage2Y, 
                    arenaZ+by4])
                cube([by2, by1, stage2Z+by1]);
            }
            
            // saber
            // Note: this is an approximation until
            //    the final design of the light
            //    saber is ready
            color(stage_trim_color) 
                translate([0, 
                    stage2Y/2, 
                    arenaZ+by4+stage2Z+by1])
                cylinder(r=saberD/2, h=saberZ);
        }
        
        // mounting hole
        translate([-fudge, 
            stage2Y/2, 
            arenaZ+stage2Hole])
        rotate([0, 90, 0])
        cylinder(r=3/8*inch/2, 
            h=halfInchPly+fudge+fudge);
    }
}

module drawStage3()
{
    difference() {    
        union()
        {
            // wall
            color(stage_wall_color)
                cube([stage3X, 
                    halfInchPly, 
                    arenaZ+by4+stage3Z]);
            
            // frame
            color(stage_trim_color) {
                translate([-by1, 
                    0, 
                    arenaZ+by4])
                cube([by1, by2, stage3Z+by1]);
                translate([0, 
                    0, 
                    arenaZ+by4+stage3Z])
                cube([stage3X, by2, by1]);
                translate([stage3X, 
                    0, 
                    arenaZ+by4])
                cube([by1, by2, stage3Z+by1]);
            }
            
            // knob
            color(stage3_knob_color) 
                translate([stage3X/2, 
                    halfInchPly, 
                    arenaZ+by4+stage3Z/2])
                rotate([-90, 0, 0])
                cylinder(r1=knobD1/2, 
                    r2=knobD2/2, 
                    h=knobThickness);
        }
        
        // mounting hole
        translate([stage3X/2, 
            halfInchPly+fudge, 
            arenaZ+by4/2])
        rotate([90, 0, 0])
        cylinder(r=3/8*inch/2, 
            h=halfInchPly+fudge+fudge);
    }
}

module drawFinalStage()
{
    difference()
    {
        // wall
        color(stage_wall_color) 
            translate([0, 0, 0]) 
            cube([halfInchPly, arenaY, finalZ]);

        // hole
        translate([-fudge, 
            arenaY/2-finalHole/2, 
            finalHoleHeight])
        cube([halfInchPly+fudge+fudge, 
            finalHole, 
            finalHole]);
    }
    
    // frame inset into hole in wall
    color(stage_trim_color) {
        translate([-halfInchPly, 
            arenaY/2-finalHole/2, 
            finalHoleHeight])
        cube([by2,finalHole-by1,by1]);
        translate([-halfInchPly, 
            arenaY/2-finalHole/2+by1, 
            finalHoleHeight+finalHole-by1])
        cube([by2,finalHole-by1,by1]);
        translate([-halfInchPly, 
            arenaY/2-finalHole/2, 
            finalHoleHeight+by1])
        cube([by2,by1,finalHole-by1]);
        translate([-halfInchPly, 
            arenaY/2+finalHole/2-by1, 
            finalHoleHeight])
        cube([by2,by1,finalHole-by1]);
    }
}

module drawArena()
{
    // Arena frame, walls and steps
    drawFloor();
    drawWalls();   
    drawStep1();
    drawStep2();
    drawStep3();
    drawStep4();
    drawStartSquare();
    
    // Stage 1
    translate(
        [by2+step3X+step2X+step1X+floorX/2+stage1X/2,
        arenaY+halfInchPly, 
        0]) 
    rotate([0, 0, 180]) 
    drawStage1();
    
    // Stage 2
    translate(
        [arenaX, 
        arenaY/2-stage2Y/2, 
        0]) 
    drawStage2();
   
   // Stage 3
   translate(
        [by2+step3X+step2X+step1X+floorX/2-stage3X/2,
        -halfInchPly, 
        0]) 
    drawStage3();    
    
    // Stage 4
    translate([-halfInchPly, 0, 0]) 
    drawFinalStage();
}


intersection()
{
   translate(
            [-(by2+step3X+step2X+step1X+floorX/2), 
            -arenaY/2, 
            0]) 
    drawArena();    
}
