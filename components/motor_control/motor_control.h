#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

int get_motor_position(int motor_id);
void get_all_motor_positions(int *positions, int num_motors);

#endif // MOTOR_CONTROL_H
