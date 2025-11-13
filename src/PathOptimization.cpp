/*
 * PathOptimization.cpp
 * Implementation of iterative path simplification.
 */

#include "PathOptimization.h"

PathOptimization::PathOptimization() {}

void PathOptimization::optimize(String &path) {
    bool changesMade;
    
    // Loop until a full pass makes no changes
    do {
        changesMade = false;
        String originalPath = path; // Make a copy to check for changes

        // Rule 1: LBR -> B
        path.replace("LBR", "B");
        // Rule 2: LBS -> R
        path.replace("LBS", "R");
        // Rule 3: RBL -> B
        path.replace("RBL", "B");
        // Rule 4: SBL -> R
        path.replace("SBL", "R");
        // Rule 5: SBS -> B
        path.replace("SBS", "B");
        // Rule 6: LBL -> S
        path.replace("LBL", "S");
        // Rule 7: RBR -> S
        path.replace("RBR", "S");
        // Rule 8: SBR -> L
        path.replace("SBR", "L");

        if (path!= originalPath) {
            changesMade = true; // If the string changed, loop again
        }

    } while (changesMade);
}