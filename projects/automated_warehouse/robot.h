#ifndef _PROJECTS_PROJECT1_ROBOT_H__
#define _PROJECTS_PROJECT1_ROBOT_H__

/**
 * A Structure representing robot
 */
struct robot {
    char* name;
    int row;      // 현재 행 위치
    int col;      // 현재 열 위치
    int required_payload;   // 적재해야 할 물건 번호
    int current_payload;    // 지금 적재된 물건 (0이면 없음)
    int l_row; 
    int l_col; 
    int f_row;
    int f_col;
    int is_finish;
};

typedef struct {
    int row;
    int col;
} Position;

typedef struct {
    int num;
    char letter;
} NumCharPair;

typedef struct {
    int row;
    int col;
} Point;

void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload, int l_row, int l_col, int f_row, int f_col, int is_finish);

Position findPosition(char c);
NumCharPair split_string(const char* str);

#endif
