/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -1855, 2553, 349, 2243, 10039, 6141, 0.6, 0);
vex::vision::signature DISC = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 5997, 7231, 6614, 1519, 2485, 2002, 3.6, 0);
vex::vision::signature SNEHA = vex::vision::signature (4, -1449, 1, -724, 9087, 12073, 10580, 5.7, 0);
vex::vision::signature MORGAN = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision1 = vex::vision (vex::PORT18, 66, BGOAL, DISC, RGOAL, SNEHA, MORGAN, SIG_6, SIG_7);
/*vex-vision-config:end*/