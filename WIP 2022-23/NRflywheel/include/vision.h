/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -2275, 1, -1137, 5957, 12935, 9446, 1.6, 0);
vex::vision::signature DISC = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 6747, 7309, 7028, 2773, 3097, 2935, 7, 0);
vex::vision::signature SNEHA = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 11, 0);
vex::vision::signature MORGAN = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision16 = vex::vision (vex::PORT16, 85, BGOAL, DISC, RGOAL, SNEHA, MORGAN, SIG_6, SIG_7);
/*vex-vision-config:end*/