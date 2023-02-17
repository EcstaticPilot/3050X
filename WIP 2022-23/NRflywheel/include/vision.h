/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -2275, 1, -1137, 5957, 12935, 9446, 1.8, 0);
vex::vision::signature DISC = vex::vision::signature (2, 4527, 5117, 4822, -4065, -3547, -3806, 2.4, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 6951, 8651, 7801, -963, 327, -318, 3.7, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision16 = vex::vision (vex::PORT16, 58, BGOAL, DISC, RGOAL, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/