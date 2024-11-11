//change this during the competition
#include "path-planner.h"

coord start(1, 0);
coord end(7, 9);
coord g1(7, 1);
coord g2(7, 5);
coord g3(1, 5);
coord g4(3, 9);
coord o1(1, 8);
coord o2(2, 1);
coord o3(2, 7);
coord o4(3, 2);
coord o5(4, 1);
coord o6(4, 3);
coord o7(4, 5);
coord o8(5, 2);
coord o9(5, 8);
coord o10(6, 3);
unsigned long duration = 60 * 1000; // in milliseconds, target time between 50 and 75s
bool stupid = false; // no pid stupid mode
bool useDelay = true; // adds delay based on calculated time, need stupid to be false