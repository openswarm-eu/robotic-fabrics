#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
//#define DEBUG
//#include <debug.h>

// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Constant Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// The CODE is scalable trough 3 variables: number of kilobots, number of grid rows, number of grid coloumns.
#define NUM_Kilobots 49                     // Number of kilobots
#define PI 3.1415926535                     // Pi
#define Num_Matrix_Rows 7                   // Number of grid rows
#define Num_Matrix_Coloumns 7           // Number of grid coloumns

#define STOP 0                              // variables used for the motion (see function: void set_motion())
#define FORWARD 1
#define LEFT 2
#define RIGHT 3



#define MINIMUM_DEGREES 85
#define MAXIMUM_DEGREES 95
#define DIFFERENCE_THRESHOLD  10
#define STRAIGHT_THRESHOLD 70
#define DIAGONAL_THRESHOLD_LOWER 100
#define DIAGONAL_THRESHOLD_UPPER  110

#define MOTION_DELAY 100

// Turning
// Assuming LINK_LENGTH (m) and ROTATION_RADIUS (m) have certain fixed values, modify as needed
#define LINK_LENGTH 0.07
#define ROTATION_RADIUS 1
#define ROTATION_DIRECTION 'R' // 'L' for left, 'R' for right

// --------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Variable Declaration --------------------------------------------
// --------------------------------------------------------------------------------------------------------------

// ---------- Message Variables ------------
message_t message;
int new_message = 0;

int composed_message = 0;                   // the firs byte is composed of two part, one is the kilobot ID and the other is the status is which the Kilobot is operating
int composed_message_rx = 0;

int message_ID = 0;
int message_distance = 0;

int ID_N_dist = 0;                          // ID of the Kilobot positioned above, on the NORD direction; variable used for the receiving messages
int ID_NE_dist = 0;
int ID_E_dist = 0;
int ID_SE_dist = 0;
int ID_S_dist = 0;
int ID_SW_dist = 0;
int ID_W_dist = 0;
int ID_NW_dist = 0;



// ---------- State Variables ------------
unsigned int State = 0;
int operating_mode_tx = 0;                  // variable transmitted and received in the composed message together with the Kilobot ID
int operating_mode_rx = 0;


// ---------- Neighbour Variables ------------
int kil_dist_matrix[9][9];          //Matrix to store the distance information of all neighbours
int neighbours_IDs_array[9] = {0};      //Matrix to store IDs of neighbours
int temp_dist = 0;

int kil_dist_N = 0;                         // variables used in the motion algorithm
int kil_dist_NE = 0;
int kil_dist_E = 0;
int kil_dist_SE = 0;
int kil_dist_S = 0;
int kil_dist_SW = 0;
int kil_dist_W = 0;
int kil_dist_NW = 0;


// ---------- Angle Variables ------------
int alpha_deg[3];                   // starting from the top RIGHT position the 90-degrees angles are ordered in clockwise
int alpha_deg_two[3];
float temp_angle_float = 0;
float dist_float[3];
float dist_float_two[3];            // in order to compute angles relative to differents triangular shapes

// ---------- Loop Variables ------------
int i = 0;
int j = 0;
int k = 0;
int l = 0;

// ---------- Move Variables ------------
unsigned int current_motion = 0;

// ---------- Time Variables ------------
unsigned long int last_timereset = 0;

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
        else if (current_motion == LEFT)
        {
            //set_color(RGB(0, 0, 3)); //blue
            spinup_motors();
            set_motors(kilo_turn_left,0);

        }
        else if (current_motion == RIGHT)
        {
            //set_color(RGB(3, 0, 3)); //pink
            spinup_motors();
            set_motors(0,kilo_turn_right);
        }
    }
}

// --------------------------------------------------------------------------------------------------------------
// --------------------------------------- Message Construction Function ----------------------------------------
// --------------------------------------------------------------------------------------------------------------

void msg_codedist_tx(uint8_t operating_mode, uint8_t N_dist, uint8_t NE_dist, uint8_t E_dist, uint8_t SE_dist, uint8_t S_dist, uint8_t SW_dist, uint8_t W_dist, uint8_t NW_dist){

    composed_message = operating_mode << 7 | kilo_uid;

    message.type = NORMAL;
    message.data[0] = composed_message;
    message.data[1] = N_dist;
    message.data[2] = NE_dist;
    message.data[3] = E_dist;
    message.data[4] = SE_dist;
    message.data[5] = S_dist;
    message.data[6] = SW_dist;
    message.data[7] = W_dist;
    message.data[8] = NW_dist;
    message.crc = message_crc(&message);

    composed_message = operating_mode << 7 | kilo_uid;

    //printf("ID = %d and N_dist is %d and NE_dist is %d and E_dist is %d and SE_dist is %d and S_dist is %d and SW_dist is %d and W_dist is %d and NW_dist is %d \n \n", kilo_uid, N_dist, NE_dist, E_dist,SE_dist,S_dist,SW_dist,W_dist,NW_dist);

}


message_t *message_tx(){

    return &message;
}

void tx_message_success(){

    msg_codedist_tx(operating_mode_tx,kil_dist_matrix[0][1],kil_dist_matrix[0][2],kil_dist_matrix[0][3],kil_dist_matrix[0][4],kil_dist_matrix[0][5],kil_dist_matrix[0][6],kil_dist_matrix[0][7],kil_dist_matrix[0][8]);
}

// --------------------------------------------------------------------------------------------------------------
// --------------------------------------- Turning functions  ----------------------------------------
// --------------------------------------------------------------------------------------------------------------

//for turning

// Function to compute R_left
double compute_R_left(double i, double j) {
    double term1 = (LINK_LENGTH / sqrt(2)) * (j + i) + ROTATION_RADIUS;
    double term2 = (LINK_LENGTH / sqrt(2)) * (j - i);
    double R_left = sqrt(term1 * term1 + term2 * term2);
    return R_left;
}

// Function to compute R_right
double compute_R_right(double i, double j) {
    double term1 = (LINK_LENGTH / sqrt(2)) * (j + i) - ROTATION_RADIUS;
    double term2 = (LINK_LENGTH / sqrt(2)) * (j - i);
    double R_right = sqrt(term1 * term1 + term2 * term2);
    return R_right;
}

// Function to compute p_left
float p_left(int i, int j) {
    double R_left_ij = compute_R_left(i, j);  // Symmetry for left
    double R_left_far = compute_R_left((Num_Matrix_Rows - 1) / 2, (Num_Matrix_Rows - 1) / 2);  // Center of grid
    return (float)(R_left_ij / R_left_far);
}

// Function to compute p_right
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

    // Setting initialised variables to 0 to avoid NaN errors in case of communication delays
    for (i = 0; i < 9; ++i){
        for (j = 0; j < 9; ++j)
        {
            kil_dist_matrix[i][j] = 0;
        }
    }

    for (i = 0; i < 3; ++i)
    {
        alpha_deg[i] = 0;
        dist_float[i] = 0;
        dist_float_two[i] = 0;
    }

    // saving the neighbours ID in the array following the proper order, in the position 0 own ID is always saved


    /*  DIAGRAM
        /----\        /----\        /----\              /----\        /----\        /----\
        |  A |========|  B |========|  C |              |  8 |========|  1 |========|  2 |
        \----/        \----/        \----/              \----/        \----/        \----/
          ||            ||            ||                  ||            ||            ||
          ||            ||            ||                  ||            ||            ||
          ||            ||            ||                  ||            ||            ||
        /----\        /----\        /----\              /----\        /----\        /----\
        |  D |========|  E |========|  F |              |  7 |========|  0 |========|  3 |
        \----/        \----/        \----/              \----/        \----/        \----/
          ||            ||            ||                  ||            ||            ||
          ||            ||            ||                  ||            ||            ||
          ||            ||            ||                  ||            ||            ||
        /----\        /----\        /----\              /----\        /----\        /----\
        |  G |========|  H |========|  I |              |  6 |========|  5 |========|  4 |
        \----/        \----/        \----/              \----/        \----/        \----/
    */

    if(kilo_uid <= Num_Matrix_Coloumns)             //If KB is on the first row (1 to n) (A to C)
    {
        if(kilo_uid == 1)                           //If first in row (A on diagram) (storing A (itself) and B,E,D)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[3] = kilo_uid + 1;
            neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
            neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
        }
        else if(kilo_uid == Num_Matrix_Coloumns)    //If last in row (C on diagram)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
            neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
            neighbours_IDs_array[7] = kilo_uid - 1;
        }
        else                                        //If any other (B on diagram)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[3] = kilo_uid + 1;
            neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
            neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
            neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
            neighbours_IDs_array[7] = kilo_uid - 1;
        }
    }
    else if(kilo_uid > Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns) //If KB is on the last row ((n^2)-n  to n^2) (G to I)
    {
        if((kilo_uid - 1) % Num_Matrix_Coloumns == 0)                   //If first in row (G on diagram) (storing G (itself) and D,E,H)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
            neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
            neighbours_IDs_array[3] = kilo_uid + 1;
        }
        else if(kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns)      //If last in row (I on diagram)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
            neighbours_IDs_array[7] = kilo_uid - 1;
            neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
        }
        else                                                            //If any other (H on diagram)
        {
            neighbours_IDs_array[0] = kilo_uid;
            neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
            neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
            neighbours_IDs_array[3] = kilo_uid + 1;
            neighbours_IDs_array[7] = kilo_uid - 1;
            neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
        }
    }
    else if((kilo_uid - 1) % Num_Matrix_Coloumns == 0)  //If KB is on the left edge (D)
    {
        neighbours_IDs_array[0] = kilo_uid;
        neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
        neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
        neighbours_IDs_array[3] = kilo_uid + 1;
        neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
        neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;

    }
    else if(kilo_uid % Num_Matrix_Coloumns == 0)        //If KB is on the right edge (F)
    {

        neighbours_IDs_array[0] = kilo_uid;
        neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
        neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
        neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
        neighbours_IDs_array[7] = kilo_uid - 1;
        neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
    }
    else                                                //Otherwise its in the centre (E)
    {
        neighbours_IDs_array[0] = kilo_uid;
        neighbours_IDs_array[1] = kilo_uid - Num_Matrix_Coloumns;
        neighbours_IDs_array[2] = kilo_uid - Num_Matrix_Coloumns + 1;
        neighbours_IDs_array[3] = kilo_uid + 1;
        neighbours_IDs_array[4] = kilo_uid + Num_Matrix_Coloumns + 1;
        neighbours_IDs_array[5] = kilo_uid + Num_Matrix_Coloumns;
        neighbours_IDs_array[6] = kilo_uid + Num_Matrix_Coloumns - 1;
        neighbours_IDs_array[7] = kilo_uid - 1;
        neighbours_IDs_array[8] = kilo_uid - Num_Matrix_Coloumns - 1;
    }

    //Logic check to ensure that any invalid Neighbour IDs are set to 0
    for (i = 0; i < 9; ++i){

        if(neighbours_IDs_array[i] < 0 || neighbours_IDs_array[i] > NUM_Kilobots){

            neighbours_IDs_array[i] = 0;
        }
    }

    if (kilo_uid == 1){                                     // LEDs turned on for 3 seconds in order to check the proper uploading of the code on the Kilobot,
                                                                    // each robot LED colour depends on the ID, so on the position, of the robot itself in the grid

        set_color(RGB(1,0,0));      // red LED

    } else if (kilo_uid < Num_Matrix_Coloumns && kilo_uid > 1){

        set_color(RGB(0,1,0));      // green LED

    } else if (kilo_uid == Num_Matrix_Coloumns){

        set_color(RGB(0,0,1));      // blue LED

    } else if ((kilo_uid - 1) % Num_Matrix_Coloumns == 0 && kilo_uid != 1 && kilo_uid < (Num_Matrix_Rows * Num_Matrix_Coloumns) - Num_Matrix_Coloumns){

        set_color(RGB(1,1,0));      // orange LED

    } else if (kilo_uid % Num_Matrix_Coloumns == 0 && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows && kilo_uid != Num_Matrix_Coloumns){

        set_color(RGB(1,0,1));      // violet LED

    } else if (kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1){

        set_color(RGB(0,1,1));      // magenta LED

    } else if (kilo_uid > (Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1) && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows){

        set_color(RGB(1,1,1));      // white LED

    } else if (kilo_uid == Num_Matrix_Coloumns * Num_Matrix_Rows){

        set_color(RGB(1,0,0));      // red LED

    } else {

        set_color(RGB(0,0,0));      // LED turned off
    }


    delay(3000);

    // start the message transmission
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.crc = message_crc(&message);

    // variable for the timer
    last_timereset = kilo_ticks;

    //Setting state to 0 (only state in current implementation)
    State = 0;


    /* Setup for turning */
    // Calculate position (x, y) in the grid based on ID
    get_coordinates(kilo_uid, Num_Matrix_Rows, &i_pos_in_grid, &j_pos_in_grid);
//    printf("Robot ID: %d at position (x, y): (%d, %d)\n", kilo_uid, i_pos_in_grid, j_pos_in_grid);

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

    if (State == 0){                        /******************************* State 0 ********************************/
                                            // in this code there is only one state, but other states can be easily added

        // ----------------------------------------------------------------------------------------------------------
        // Computing the average distance between node i and j (from stored values of dist(i,j) and dist(j,i)
        for (i = 0; i < 9; ++i)
        {
            for (j = i; j < 9; ++j)
            {
                if(kil_dist_matrix[i][j] != 0 && kil_dist_matrix[j][i] == 0)
                {
                    kil_dist_matrix[j][i] = kil_dist_matrix[i][j];
                }
                else if(kil_dist_matrix[j][i] != 0 && kil_dist_matrix[i][j] == 0)
                {
                    kil_dist_matrix[i][j] = kil_dist_matrix[j][i];
                }
                else if(kil_dist_matrix[i][j] != 0 && kil_dist_matrix[j][i] != 0)   // compute the average value
                {
                    temp_dist = ((float)kil_dist_matrix[i][j] + (float)kil_dist_matrix[j][i]) / 2.0 + 0.5;      // approximation by excess
                    kil_dist_matrix[i][j] = (int)temp_dist;
                    kil_dist_matrix[j][i] = (int)temp_dist;
                }
            }
        }

        // ----------------------------------------------------------------------------------------------------------
        // Comptuing the angles based on the distance information - where two angles can be compted they are (in centre)
        // Depending on the position of the Kilobot in the grid, different distances are needed to compute the angles

        if (kilo_uid == 1)                                              // kiloID 1 (in a 3x3 grid)
        {
            dist_float[0] = (float)kil_dist_matrix[5][3];
            dist_float[1] = (float)kil_dist_matrix[0][5];
            dist_float[2] = (float)kil_dist_matrix[0][3];
        }
        else if (kilo_uid < Num_Matrix_Coloumns && kilo_uid > 1)        // kiloID 2 (in a 3x3 grid)
        {
            dist_float[0] = (float)kil_dist_matrix[5][3];
            dist_float[1] = (float)kil_dist_matrix[0][5];
            dist_float[2] = (float)kil_dist_matrix[0][3];

            dist_float_two[0] = (float)kil_dist_matrix[7][5];
            dist_float_two[1] = (float)kil_dist_matrix[0][7];
            dist_float_two[2] = (float)kil_dist_matrix[0][5];
        }
        else if(kilo_uid == Num_Matrix_Coloumns * Num_Matrix_Rows)      // kiloID 9 (in a 3x3 grid)
        {
            dist_float[0] = (float)kil_dist_matrix[7][1];
            dist_float[1] = (float)kil_dist_matrix[0][7];
            dist_float[2] = (float)kil_dist_matrix[0][1];
        }
        else if(kilo_uid == Num_Matrix_Coloumns)                        // kiloID 3 (in a 3x3 grid)
        {
            dist_float[0] = (float)kil_dist_matrix[7][5];
            dist_float[1] = (float)kil_dist_matrix[0][7];
            dist_float[2] = (float)kil_dist_matrix[0][5];
        }
        else if (kilo_uid % Num_Matrix_Coloumns == 0 && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows && kilo_uid != Num_Matrix_Coloumns)
        {                                                               // kiloID 6 (in a 3x3 grid)
            dist_float[0] = (float)kil_dist_matrix[7][5];
            dist_float[1] = (float)kil_dist_matrix[0][7];
            dist_float[2] = (float)kil_dist_matrix[0][5];

            dist_float_two[0] = (float)kil_dist_matrix[7][1];
            dist_float_two[1] = (float)kil_dist_matrix[0][7];
            dist_float_two[2] = (float)kil_dist_matrix[0][1];
        }
        else if (kilo_uid > (Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1) && kilo_uid != Num_Matrix_Coloumns * Num_Matrix_Rows)
        {                                                               // kiloID 8 (in a 3x3 grid)
            dist_float[0] = (float)kil_dist_matrix[3][1];
            dist_float[1] = (float)kil_dist_matrix[0][3];
            dist_float[2] = (float)kil_dist_matrix[0][1];

            dist_float_two[0] = (float)kil_dist_matrix[7][1];
            dist_float_two[1] = (float)kil_dist_matrix[0][7];
            dist_float_two[2] = (float)kil_dist_matrix[0][1];
        }
        else if (kilo_uid == Num_Matrix_Rows * Num_Matrix_Coloumns - Num_Matrix_Coloumns + 1)
        {                                                               // kiloID 7 (in a 3x3 grid)
            dist_float[0] = (float)kil_dist_matrix[3][1];
            dist_float[1] = (float)kil_dist_matrix[0][3];
            dist_float[2] = (float)kil_dist_matrix[0][1];
        }
        else if ((kilo_uid - 1) % Num_Matrix_Coloumns == 0 && kilo_uid != 1 && kilo_uid < (Num_Matrix_Rows * Num_Matrix_Coloumns) - Num_Matrix_Coloumns)
        {                                                               // kiloID 4 (in a 3x3 grid)
            dist_float[0] = (float)kil_dist_matrix[3][1];
            dist_float[1] = (float)kil_dist_matrix[0][3];
            dist_float[2] = (float)kil_dist_matrix[0][1];

            dist_float_two[0] = (float)kil_dist_matrix[5][3];
            dist_float_two[1] = (float)kil_dist_matrix[0][5];
            dist_float_two[2] = (float)kil_dist_matrix[0][3];
        }
        else
        {                                                               // kiloID 5 (in a 3x3 grid)
            dist_float[0] = (float)kil_dist_matrix[5][3];
            dist_float[1] = (float)kil_dist_matrix[0][5];
            dist_float[2] = (float)kil_dist_matrix[0][3];

            dist_float_two[0] = (float)kil_dist_matrix[7][1];
            dist_float_two[1] = (float)kil_dist_matrix[0][7];
            dist_float_two[2] = (float)kil_dist_matrix[0][1];
        }


        // ----------------------------------
        kil_dist_N = kil_dist_matrix[0][1];                             // using an easy name for the distances
        kil_dist_NE = kil_dist_matrix[0][2];
        kil_dist_E = kil_dist_matrix[0][3];
        kil_dist_SE = kil_dist_matrix[0][4];
        kil_dist_S = kil_dist_matrix[0][5];
        kil_dist_SW = kil_dist_matrix[0][6];
        kil_dist_W = kil_dist_matrix[0][7];
        kil_dist_NW = kil_dist_matrix[0][8];


        // Generate a random number between 0 and 1
        float p = (float)rand() / RAND_MAX;

        // Compare random number with probability and move accordingly
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
// ----------------------------------------- Message Recieve Function -------------------------------------------
// --------------------------------------------------------------------------------------------------------------

void message_rx(message_t *m, distance_measurement_t *distance_measurement){

    if (m->type == 120) { // initial identification

        int id = (m->data[0] << 8) | m->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }

    } else {

        new_message = 1;

        composed_message_rx = m->data[0];

        operating_mode_rx = composed_message_rx >> 7;       // only the first bit is used to transmit the operation mode
        message_ID = composed_message_rx & 127;             // & 01111111 -> considering the latest seven bits
        ID_N_dist = m->data[1];                                     // receiving distances from a near Kilobot ad its neighbourhood
        ID_NE_dist = m->data[2];
        ID_E_dist = m->data[3];
        ID_SE_dist = m->data[4];
        ID_S_dist = m->data[5];
        ID_SW_dist = m->data[6];
        ID_W_dist = m->data[7];
        ID_NW_dist = m->data[8];
        message_distance = estimate_distance(distance_measurement);


        if (message_ID != 0 && message_distance != 0){              // filtering the distances detected, saving them following the proper order in the matrix of distances

            for (l = 1; l < 9; ++l){

                if(neighbours_IDs_array[l] == message_ID){

                    kil_dist_matrix[0][l] = message_distance;

                    break;
                }
            }

            if(message_ID == kilo_uid - 1)              // saving the distances of the neighbourhood transmitted in the message
            {
                kil_dist_matrix[7][8] = ID_N_dist;      // distance between the Kilobot ID, from which the message is received, and its neighbour at the NORD.
                kil_dist_matrix[7][1] = ID_NE_dist;
                kil_dist_matrix[7][0] = ID_E_dist;
                kil_dist_matrix[7][5] = ID_SE_dist;
                kil_dist_matrix[7][6] = ID_S_dist;
            }
            else if(message_ID == kilo_uid + 1)
            {
                kil_dist_matrix[3][2] = ID_N_dist;
                kil_dist_matrix[3][1] = ID_NW_dist;
                kil_dist_matrix[3][0] = ID_W_dist;
                kil_dist_matrix[3][5] = ID_SW_dist;
                kil_dist_matrix[3][4] = ID_S_dist;
            }
            else if(message_ID == kilo_uid - Num_Matrix_Coloumns)
            {
                kil_dist_matrix[1][2] = ID_E_dist;
                kil_dist_matrix[1][3] = ID_SE_dist;
                kil_dist_matrix[1][0] = ID_S_dist;
                kil_dist_matrix[1][7] = ID_SW_dist;
                kil_dist_matrix[1][8] = ID_W_dist;
            }
            else if(message_ID == kilo_uid + Num_Matrix_Coloumns)
            {
                kil_dist_matrix[5][0] = ID_N_dist;
                kil_dist_matrix[5][3] = ID_NE_dist;
                kil_dist_matrix[5][4] = ID_E_dist;
                kil_dist_matrix[5][6] = ID_W_dist;
                kil_dist_matrix[5][7] = ID_NW_dist;
            }
            else if(message_ID == kilo_uid - Num_Matrix_Coloumns - 1)
            {
                kil_dist_matrix[8][1] = ID_E_dist;
                kil_dist_matrix[8][0] = ID_SE_dist;
                kil_dist_matrix[8][7] = ID_S_dist;
            }
            else if(message_ID == kilo_uid - Num_Matrix_Coloumns + 1)
            {
                kil_dist_matrix[2][3] = ID_S_dist;
                kil_dist_matrix[2][0] = ID_SW_dist;
                kil_dist_matrix[2][1] = ID_W_dist;
            }
            else if(message_ID == kilo_uid + Num_Matrix_Coloumns - 1)
            {
                kil_dist_matrix[6][7] = ID_N_dist;
                kil_dist_matrix[6][0] = ID_NE_dist;
                kil_dist_matrix[6][5] = ID_E_dist;
            }
            else if(message_ID == kilo_uid + Num_Matrix_Coloumns + 1)
            {
                kil_dist_matrix[4][3] = ID_N_dist;
                kil_dist_matrix[4][0] = ID_NW_dist;
                kil_dist_matrix[4][5] = ID_W_dist;
            }
            else
            {
                // the message has come from a neighbour of a neighbour
            }
        }
    }
}

// --------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Main ----------------------------------------------------
// --------------------------------------------------------------------------------------------------------------

int main(){

    kilo_init();  // Initialize Kilobot
    //debug_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_message_tx_success = tx_message_success;
    kilo_start(setup, loop);

    return 0;
}


