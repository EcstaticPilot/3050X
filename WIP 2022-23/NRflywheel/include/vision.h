/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -993, 3185, 1096, 5569, 9305, 7438, 2.2, 0);
vex::vision::signature DISC = vex::vision::signature (2, 4527, 5117, 4822, -4065, -3547, -3806, 2.4, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 2979, 6473, 4726, 1143, 2867, 2005, 2.3, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, -2049, -655, -1352, -729, 2185, 728, 0, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 6.5, 0);
vex::vision Vision16 = vex::vision (vex::PORT16, 53, BGOAL, DISC, RGOAL, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/