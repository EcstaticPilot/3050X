/*vex-vision-config:begin*/

#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -3121, -1755, -2438, 7959, 11423, 9691, 2.8, 0);
vex::vision::signature DISC = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 5997, 7231, 6614, 1519, 2485, 2002, 6.5, 0);
vex::vision::signature SNEHA = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature MORGAN = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision16 = vex::vision (vex::PORT16, 150, BGOAL, DISC, RGOAL, SNEHA, MORGAN, SIG_6, SIG_7);
/*vex-vision-config:end*/