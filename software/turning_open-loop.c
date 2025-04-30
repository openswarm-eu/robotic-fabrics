#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Constant Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// ---------- MODIFY the following parameters to match the size of the system ----------
#define Num_Matrix_Rows 7            // Number of grid rows
#define Num_Matrix_Coloumns 7        // Number of grid coloumns
#define LINK_LENGTH 0.07             // The center-to-center distance between the robots (m)
#define ROTATION_RADIUS 1            // The rotation radius of the robot-fabric (m)
#define ROTATION_DIRECTION 'L'       // 'L' for left rotation, 'R' for right rotation
// -------------------------------------------------------------------------------------

#define STOP 0                       // variables used for the motion (see function: void set_motion())
#define FORWARD 1

#define MOTION_DELAY 100

// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Variable Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// ---------- State Variables -----------
unsigned int State = 0;

// ---------- Move Variables ------------
unsigned int current_motion = STOP;

// ---------- Turning Variables ------------
int i_pos_in_grid, j_pos_in_grid;
float prob_left, prob_right;

// --------------------------------------------------------------------------------------------------------------
// ----------------------------------------------- Motor Function -----------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void set_motion(unsigned int new_motion){

    if(current_motion != new_motion)
    {
        current_motion = new_motion;
        if (current_motion == STOP)
        {
            set_color(RGB(1, 0, 0)); //red
            set_motors(0,0);
        }
        else if(current_motion == FORWARD)
        {
            set_color(RGB(0, 1, 0)); //green
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
    }
}

// --------------------------------------------------------------------------------------------------------------
// --------------------------------------- Turning functions  ----------------------------------------
// --------------------------------------------------------------------------------------------------------------

//for turning

// Function to compute the radius for turning left
double compute_R_left(double i, double j) {
    double term1 = (LINK_LENGTH / sqrt(2)) * (j + i) + ROTATION_RADIUS;
    double term2 = (LINK_LENGTH / sqrt(2)) * (j - i);
    double R_left = sqrt(term1 * term1 + term2 * term2);
    return R_left;
}

// Function to compute the radius for turning right
double compute_R_right(double i, double j) {
    double term1 = (LINK_LENGTH / sqrt(2)) * (j + i) - ROTATION_RADIUS;
    double term2 = (LINK_LENGTH / sqrt(2)) * (j - i);
    double R_right = sqrt(term1 * term1 + term2 * term2);
    return R_right;
}

// Function to compute the probability to move forward when turning left
float p_left(int i, int j) {
    double R_left_ij = compute_R_left(i, j);  // Symmetry for left
    double R_left_far = compute_R_left((Num_Matrix_Rows - 1) / 2, (Num_Matrix_Rows - 1) / 2);  // Center of grid
    return (float)(R_left_ij / R_left_far);
}

// Function to compute the probability to move forward when turning right
float p_right(int i, int j) {
    double R_right_ij = compute_R_right(i, j);
    double R_right_far = compute_R_right(-(Num_Matrix_Rows - 1) / 2, -(Num_Matrix_Rows - 1) / 2);  // Bottom-left corner of grid
    return (float)(R_right_ij / R_right_far);
}

// Function to return the x and y coordinates for a given id and grid size n
void get_coordinates(int id, int n, int *x, int *y) {
    int k = (n - 1) / 2;  // Calculate the offset to center the grid at (0, 0)

    int row = (id - 1) / n;  // Calculate the row index
    int col = (id - 1) % n;  // Calculate the column index

    *x = col - k;  // Calculate the x-coordinate
    *y = k - row;  // Calculate the y-coordinate
}

// Initialize random number generator
void init_random(int seed) {
    srand(seed);
}

// --------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- Initialization ------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void setup(){

    /*
    * The kilo_uid of a Kilobot in the robot-fabric must be initialized from left-to-right, top-to-bottom as shown below.
    * The robot-fabric moves in the direction of the robot with kilo_uid = 1.
    *
    * - Example: 3x3
    *
    *
    *                            Front
    *
    *                             /\
    *                             ||
    *
    *                            [1]
    *                           /   \
    *                        [4]     [2]
    *                       /   \   /   \
    *                    [7]     [5]     [3]
    *                       \   /   \   /
    *                        [8]     [6]
    *                           \   /
    *                            [9]
    *
    *                            Back
    */

    //Setting state to 0 (only state in current implementation)
    State = 0;

    /* Setup for turning */
    // Calculate position (x, y) in the grid based on ID
    get_coordinates(kilo_uid, Num_Matrix_Rows, &i_pos_in_grid, &j_pos_in_grid);

    // Initialize random number generator with robot's unique ID
    init_random(kilo_uid);

    // Calculate probabilities for moving left and right
    if(ROTATION_DIRECTION == 'L') {
        prob_left = p_left(i_pos_in_grid, j_pos_in_grid);
    } else if(ROTATION_DIRECTION == 'R') {
        prob_right = p_right(i_pos_in_grid, j_pos_in_grid);
    }

}

// --------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Main Loop --------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void loop(){

    if (State == 0){

        // Generate a random number between 0 and 1
        float p = (float)rand() / RAND_MAX;

        // Compare the random number with the pre-computed probability and move accordingly
        float prob;
        if(ROTATION_DIRECTION == 'L') {
            prob = prob_left;
        } else if(ROTATION_DIRECTION == 'R') {
            prob = prob_right;
        }

        if (p < prob) {
            set_motion(FORWARD);
            delay(MOTION_DELAY);
        } else {
            set_motion(STOP);
            delay(MOTION_DELAY);
        }
    }
}

// --------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Main ----------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

int main(){

    kilo_init();  // Initialize Kilobot
    kilo_start(setup, loop);
    return 0;
}