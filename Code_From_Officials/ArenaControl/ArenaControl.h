/*
 * Arduino connection map:
 *   D0:  (O) Rx (currently unused)
 *   D1:  (I) Tx (currently unused)
 *   D2:  (I) vibration sensor (IRQ)
 *   D3:  (I) quadrature channel B (IRQ)                                                                                                                                                                    
 *   D4:  (I) quadrature channel A
 *   D5:  (I) quadrature pushbutton (currently unused)
 *   D6:  (O) neopixel stick data
 *   D7:  (O) LED enable (active low)
 *   D8:  (O) LED red (cathode)
 *   D9:  (O) LED green (cathode)
 *   D10: (O) LED blue (cathode)
 *   D11: unused (routed to external I/O header)
 *   D12: unused (routed to external I/O header)
 *   D13: (O) magnetic coil (and UNO on-board LED)
 *   GND: neopixel, vibration, quadrature common
 *   5v:  neopixel, LED common (anode)
 *
 * I encourage everyone to read through the code and help me identify any
 *    issues. This will make the arena electronics better for both me,
 *    your team, and all the other teams.
 * Anyone who identifies any issues now (before it is complete) will have 
 *    their name listed in the collaboration section below. Once the code
 *    is listed as officially complete (expected date is January 1), will
 *    have their name listed and will received $5 gift (handed out in 
 *    Charlotte at Southeastcon - you must be present). 
 * So you can help me find the issues now and get your name listed, or 
 *    wait and hope to get your name and cash.
 * Note that only the first person to find each issue is listed (and
 *    optionally gets the cash).
 * Any bug already listed in the above known issues list are not eligible.
 *
 * COLLABORATORS / HONORS (for helping identify new issues in the software):
 *    YOUR NAME COULD BE HERE FOR EVERYONE TO SEE!
 *
 * Pete Soper - SDevCS, LLC
 * Koty Turner - University of Alabama robotics team
 * Fiona Popp - University of NC Asheville robotics team
 * Brandon Simon - Virginia COmmonwealth University
 */

/* Number of milliseconds in a second */
#define MSECS            1000

/* Number of seconds in countdown before start of competition */
#define COUNTDOWN_TIME   3

/* Length of match runtime 4 minutes, plus allow the countdown time */
#define MATCH_RUNTIME    ((4L*60L+COUNTDOWN_TIME)*MSECS)

