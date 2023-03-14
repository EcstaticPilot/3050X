/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -2209, 2317, 54, 2249, 9599, 5924, 0.8, 0);
vex::vision::signature DISC = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 7489, 7877, 7683, 937, 1261, 1099, 11, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 6.5, 0);
vex::vision Vision16 = vex::vision (vex::PORT16, 65, BGOAL, DISC, RGOAL, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/