#include "projects/automated_warehouse/aw_message.h"
#include "threads/malloc.h"
#include <stdlib.h>
#include "threads/interrupt.h"
#include "projects/automated_warehouse/aw_thread.h"
#include "projects/automated_warehouse/aw_manager.h"

struct message_box* boxes_from_central_control_node = NULL;
struct message_box* boxes_from_robots = NULL;

/* 초기화: num_robots개 박스 동적 할당 및 초기화 */	
void message_boxes(int n) {
    boxes_from_central_control_node = malloc(sizeof(struct message_box) * (n+1));
    boxes_from_robots = malloc(sizeof(struct message_box) * (n+1));

    for (int i = 0; i < n; i++) {
        boxes_from_central_control_node[i].msg.row = ROW_W;
        boxes_from_central_control_node[i].msg.col = COL_W;
        boxes_from_central_control_node[i].dirtyBit = 1; 

        boxes_from_robots[i].msg.row = ROW_W;
        boxes_from_robots[i].msg.col = COL_W;
        boxes_from_robots[i].dirtyBit = 0;
    }
}

void message_from_robots(int idx, struct message_box* msg_box, int robot_row, int robot_col) {
    msg_box->msg.row = robot_row;
    msg_box->msg.col = robot_col;
    msg_box->dirtyBit = 1;
}

void message_from_center(int idx, struct message_box* msg_box, int next_row, int next_col) {
    msg_box->msg.row = next_row;
    msg_box->msg.col = next_col;
    msg_box->dirtyBit = 1;
}

