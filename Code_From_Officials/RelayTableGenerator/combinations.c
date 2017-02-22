
#include <stdio.h>

/*
 * Simple program to calculate and print the permutations of a 16
 *   relay configuration to create the component combinations for
 *   the Southeastcon 2017 hardware competition
 *
 * Author: Rodney Radford
 *
 * It is assumed the components are connected as below, yet any
 *    valid connection is valid as long as they follow the rule 
 *    that each can connect to only 4 pads, and they each pick
 *    a unique pad with which they cannot connect).
 *
 *  D - diode (skips 5)
 *      (also use 'd' to indicate diode in opposite polarity)
 *      can connect to 1, 2, 3, 4
 *  I = inductor (skips 4)
 *      can connect to pads 1, 2, 3, 5
 *  R = resistor (skips 3)
 *      can connect to pads 1, 2, 4, 5
 *  C = capacitor (skips 2)
 *      can connect to pads 1, 3, 4, 5 (skips 2)
 *  W = wire (skips 1)
 *      can connect to pads 2, 3, 4, 5
 */

// Set to 1 to generate B01...1 numbers, or 0 to generate 0b01....1 numbers
#define GENERATE_ARDUINO_B16_FORMAT  0

// Number of valid combinations found
int count = 0;

#define BIT(x) (1 << x)
#define BIT_PATTERN_GENERATOR(top,right,left)  { 0, BIT(right), BIT(left), BIT(top) | BIT(right), BIT(top) | BIT(left) }

int d_bit_patterns[] = BIT_PATTERN_GENERATOR( 1, 2, 3);
int i_bit_patterns[] = BIT_PATTERN_GENERATOR( 4, 5, 6);
int r_bit_patterns[] = BIT_PATTERN_GENERATOR( 7, 8, 9);
int c_bit_patterns[] = BIT_PATTERN_GENERATOR(10,11,12);
int w_bit_patterns[] = BIT_PATTERN_GENERATOR(13,14,15);

#define D_d_relay 16

int d_valid[] = { 1, 2, 3, 4, 0 };
int i_valid[] = { 1, 2, 3, 0, 5 };
int c_valid[] = { 1, 2, 0, 4, 5 };
int r_valid[] = { 1, 0, 3, 4, 5 };
int w_valid[] = { 0, 2, 3, 4, 5 };



/*
 * Verify if the pad placement for each component is valid
 */
int valid_combination(int d, int i, int r, int c, int w)
{
    // If the relay matrix cannot reach, obviously invalid
    if ((d_valid[d] == 0) ||
        (i_valid[i] == 0) ||
        (r_valid[r] == 0) ||
        (c_valid[c] == 0) ||
        (w_valid[w] == 0)) 
       return 0;

    // Also invalid if any are connecting to the same pad
    // Simplest test - only ignore comparisons to self, even
    //   though I still compare redundant X==Y and Y==X
    //   D       I       R       C       W
    if (        d==r || d==c || d==c || d==w ||    // D
        i==d         || i==r || i==c || i==w ||    // I
        r==d || r==i         || r==c || r==w ||    // R
        c==d || c==i || c==r         || c==w ||    // C
        w==d || w==r || w==c || w==c)              // W
       return 0;

    // can reach, and no duplicates, so must be valid
    return 1;
}

/*
 * Unfortunately C does NOT have a binary print format, so we have
 *    to generate our own.  Admittedly these numbers could be printed
 *    in hex, but it is easier to visually see which relays should
 *    be energized in a binary format.
 * The Arduino binary number format begins with a capital B, while
 *    standard GCC format is 0b (does Arduino accept this too?)
 */
char *B16_format(int n) {
   static char buffer[32];
   char *cptr = buffer;
   int i;

#if GENERATE_ARDUINO_B16_FORMAT
   *cptr++ = 'B';
#else
   *cptr++ = '0';
   *cptr++ = 'b';
#endif

   for (i=0; i < 16; i++)
      *cptr++ = (n & BIT(16-i)) ? '1' : '0';
   *cptr++ = '\0';

   return buffer;
}

/* 
 * If the combination is valid, print it out
 *
 * NOTE: This could be modified to print out an array of 16
 *       bit numbers to compile into the Arduino code to feed
 *       directly to the relays
 */
void print_combination(int d, int i, int r, int c, int w)
{
    char buffer[5+1];
    int  bit_pattern;

    // human readable version of the pattern
    buffer[6] = '\0';
    buffer[d] = 'D';
    buffer[i] = 'I';
    buffer[r] = 'R';
    buffer[c] = 'C';
    buffer[w] = 'W';

    // relay bit pattern (16-bit value actually sent to relay board)
    bit_pattern = d_bit_patterns[d_valid[d]] |
                  i_bit_patterns[i_valid[i]] |
                  c_bit_patterns[c_valid[c]] |
                  r_bit_patterns[r_valid[r]] |
                  w_bit_patterns[w_valid[w]];

    printf("     { %s, %d, %d, %d, %d, %d }, // %s - #%d\n", 
            B16_format(bit_pattern), w+1, r+1, c+1, i+1, d+1, buffer, count);
    count++;

    buffer[d] = 'd';
    bit_pattern |= BIT(D_d_relay);
    printf("     { %s, %d, %d, %d, %d, %d }, // %s - #%d\n", 
            B16_format(bit_pattern), w+1, r+1, c+1, i+1, d+1, buffer, count);
    count++;
}


void print_header() {
   printf("/*\n");
   printf(" * Southeastcon 2017 Stage 1 relay tree control patterns\n");
   printf(" *\n");
   printf(" * Table consists of binary pattern to the 16 relays, followed\n");
   printf(" *    by the number of turns required at stage 3\n");
   printf(" *\n");
   printf(" */\n\n");
}

int main(int argc, char **argv) 
{
   int d, i, r, c, w;

   print_header();
   printf("int relay_tree[][6] = {\n");

   // Blindly iterate over all combinations, testing if valid
   //    and only printing (and counting) the valid ones
   for (d=0; d<5; d++)
      for (i=0; i < 5; i++)
          for (r=0; r < 5; r++)
              for (c=0; c < 5; c++)
                  for (w=0; w < 5; w++)
                     if (valid_combination(d,i,r,c,w))
                        print_combination(d,i,r,c,w);
   
   printf("};\n");
   return 0;
}


