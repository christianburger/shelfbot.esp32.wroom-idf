#include "motor_control.h"

// Get the position of a single motor (dummy data)
int get_motor_position(int motor_id) {
    return 100 * motor_id;
}

// Get the positions of all motors (dummy data)
void get_all_motor_positions(int *positions, int num_motors) {
    for (int i = 0; i < num_motors; i++) {
        positions[i] = get_motor_position(i + 1);
    }
}
