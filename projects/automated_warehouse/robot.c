#include <string.h>
#include <stdlib.h>
#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 */

void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload, int l_row, int l_col, int f_row, int f_col, int is_finish) {
    _robot->name = name;
    _robot->row = row;
    _robot->col = col;
    _robot->required_payload = required_payload;
    _robot->current_payload = current_payload;
    _robot->l_row = l_row;
    _robot->l_col = l_col;
    _robot->f_row = f_row;
    _robot->f_col = f_col;
    _robot->is_finish = is_finish;
}

// 맵에서 특정 문자의 위치를 찾는 함수
Position findPosition(char c) {
    Position pos;
    extern char map_draw_default[7][7]; // 외부에서 정의된 맵 참조
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            if (map_draw_default[i][j] == c) {
                pos.row = i;
                pos.col = j;
                return pos;
            }
        }
    }
    
    // 찾지 못한 경우 기본값 반환
    pos.row = -1;
    pos.col = -1;
    return pos;
}

// "2A"와 같은 문자열을 {2, 'A'}로 분리하는 함수
NumCharPair split_string(const char* str) {
    NumCharPair pair;
    pair.num = str[0] - '0';  // 문자를 정수로 변환
    pair.letter = str[1];
    return pair;
}
