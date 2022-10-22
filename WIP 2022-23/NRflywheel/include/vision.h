/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BGOAL = vex::vision::signature (1, -1855, 2553, 349, 2243, 10039, 6141, 1, 0);
vex::vision::signature SIG_2 = vex::vision::signature (2, 1681, 4463, 3072, -915, 373, -271, 1.2, 0);
vex::vision::signature RGOAL = vex::vision::signature (3, 3989, 11871, 7930, 93, 1417, 755, 1.1, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, -799, 885, 43, -507, 799, 146, 1.4, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision1 = vex::vision (vex::PORT18, 61, BGOAL, SIG_2, RGOAL, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/