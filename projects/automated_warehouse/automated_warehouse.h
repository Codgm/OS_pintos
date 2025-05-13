#ifndef __PROJECTS_AUTOMATED_WAREHOUSE_H__
#define __PROJECTS_AUTOMATED_WAREHOUSE_H__

void run_automated_warehouse(char **argv);
void cnt_thread(void* aux);
void robots_thread(void* aux);
void initialize_obstacle_map();
int determine_next_move(int cur_row, int cur_col, int target_row, int target_col, int reserved);
int bfs(int cur_row, int cur_col, int target_row, int target_col);
int is_within_bounds(int row, int col);
char* create_string_copy(const char* s);


#endif 
