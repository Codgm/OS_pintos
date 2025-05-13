#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"
#include "projects/automated_warehouse/aw_thread.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_manager.h"

#define ROW 7
#define COL 7

#define ROW_S 5
#define COL_S 5

int n;
unsigned int step;
struct robot* robots;
struct message_box* boxes_from_central_control_node;
struct message_box* boxes_from_robots;

int dr[4] = {-1, 0, 1, 0}; // 상하좌우
int dc[4] = {0, -1, 0, 1};
int maps[ROW][COL];
int s_location_occupied = 0;

void initialize_obstacle_map() {
    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            if (map_draw_default[i][j] == 'X' || (map_draw_default[i][j] >= '0' && map_draw_default[i][j] <= '9')) {
                maps[i][j] = 1;
            } else {
                maps[i][j] = 0;
            }
        }
    }
};

char* create_string_copy(const char* s) {
    if (s == NULL) return NULL;

    size_t size = strlen(s) + 1;
    char* new_str = malloc(size);
    if (new_str == NULL) return NULL;

    strlcpy(new_str, s, size);
    return new_str;
}


int is_within_bounds(int row, int col) {
    return row >= 0 && row < ROW && col >= 0 && col < COL;
}

int determine_next_move(int robot_idx, int cur_row, int cur_col, int target_row, int taget_col, int reserved[ROW][COL]) {
    int min_distance = 1000;
    int next_pos = 4;

    // 일반 경로 탐색
    for (int i = 0; i < 4; i++) {
        int nr = cur_row + dr[i];
        int nc = cur_col + dc[i];
        
        // 만약 다음 지점이 목표 지점이라면, 그 방향으로 이동
        if (nr == target_row && nc == taget_col) {
            return i;
        }

        if (is_within_bounds(nr, nc) && maps[nr][nc] == 0 && !reserved[nr][nc]) {
            // 최단 거리 계산
            int distance = bfs(robot_idx, nr, nc, target_row, taget_col);
            if (distance < min_distance) {
                min_distance = distance;
                next_pos = i;
            }

        }
    }

    return next_pos;
}

int bfs(int robot_idx, int cur_row, int cur_col, int target_row, int target_col) {
        int visited[ROW][COL] = {0}; // 방문 배열 초기화
        int distance[ROW][COL] = {0}; // 거리 저장 배열

        // 큐 초기화
        Point queue[ROW * COL];
        int front = 0, rear = 0;

        // 시작점을 큐에 삽입 후, 방문 표시
        queue[rear++] = (Point){cur_row, cur_col};
        visited[cur_row][cur_col] = 1;

        while (front < rear) {
                // 큐의 맨 앞에 있는 요소 꺼내기
                Point cur = queue[front++];

                // 만약 목표 지점에 도달했다면, 거리 반환
                if (cur.row == target_row && cur.col == target_col) {
                        return distance[target_row][target_col];
                }

                for (int i = 0; i < 4; i++) {
                        int nr = cur.row + dr[i];
                        int nc = cur.col + dc[i];
                        
                        // 만약 다음 지점이 목표 지점이라면, 거리 반환 (이게 없으면, 진입 불가)
                        if (nr == target_row && nc == target_col) {
                            return distance[cur.row][cur.col] + 1;
                        }


                        if (is_within_bounds(nr, nc) && maps[nr][nc] != 1 && !visited[nr][nc]) {
                            visited[nr][nc] = 1;
                            distance[nr][nc] = distance[cur.row][cur.col] + 1;
                            queue[rear++] = (Point){nr, nc};
                        }
                }
        }
        return 1000;
}


/* 중앙 제어 노드 스레드 */
void cnt_thread() {
    while (1) {
        int is_full = check_blocked_thread_full(n);  // 모든 로봇이 block 되었는지 확인
        int final_cnt = 0;
        // 모든 로봇이 도착했는지 확인
        for (int i = 0; i < n; i++) {
            if (robots[i].is_finish == 1) {
                final_cnt++;
            }
        }
        // 모든 로봇이 도착했다면 종료
        if (final_cnt == n) {
            printf("\n--------- Mission Complete ---------\n");
            break;
        }
        if (is_full) {
            print_map(robots, n);  // 현재 상태 시각화
            // 이동 충돌 방지용 예약 배열
            int reserved[ROW][COL] = {0};
            
            // 중요: 모든 로봇의 현재 위치를 먼저 reserved에 표시
            for (int i = 0; i < n; i++) {
                if (!robots[i].is_finish) {  // 완료되지 않은 로봇만 고려
                    int cur_row = boxes_from_robots[i].msg.row;
                    int cur_col = boxes_from_robots[i].msg.col;
                    reserved[cur_row][cur_col] = 1;  // 현재 위치 점유 표시
                }
            }
            
            // S 위치가 현재 점유되어 있다면 예약 배열에도 표시
            if (s_location_occupied) {
                reserved[ROW_S][COL_S] = 1;
            }
            
            for (int i = 0; i < n; i++) {
                if (boxes_from_robots[i].dirtyBit == 1) {
                    int cur_row = boxes_from_robots[i].msg.row;
                    int cur_col = boxes_from_robots[i].msg.col;
                    if (robots[i].is_finish == 1) {
                        message_from_center(i, &boxes_from_central_control_node[i], cur_row, cur_col);
                        boxes_from_robots[i].dirtyBit = 0;
                        continue;
                    }
                    
                    // 목적지 설정 (적재 or 하역)
                    int target_row, target_col;
                    if (robots[i].current_payload == 1) {
                        target_row = robots[i].f_row;  // 하역 장소
                        target_col = robots[i].f_col;
                    } else {
                        target_row = robots[i].l_row;  // 적재 장소
                        target_col = robots[i].l_col;
                    }

		    if (cur_row == robots[i].l_row && cur_col == robots[i].l_col && robots[i].current_payload == 0) {
                        robots[i].current_payload = 1;
                    }
                    
                    // If reached final point and is loaded, mark as finished
                    if (cur_row == robots[i].f_row && cur_col == robots[i].f_col && robots[i].current_payload == 1) {
                        robots[i].is_finish = 1;
                    }
                    
                    int direction = 4;  // 기본값: 제자리 유지
                    int next_row = cur_row;
                    int next_col = cur_col;
                    
                    // 완료되지 않은 로봇만 이동 경로 계산
                    if (!robots[i].is_finish) {
                        // 현재 위치는 비워질 예정이므로 reserved에서 제거
                        reserved[cur_row][cur_col] = 0;
                        
                        direction = determine_next_move(i, cur_row, cur_col, target_row, target_col, reserved);
                        
                        if (direction < 4) {
                            next_row = cur_row + dr[direction];
                            next_col = cur_col + dc[direction];
                            
                            bool is_moving_to_S = (next_row == ROW_S && next_col == COL_S);
                            bool is_leaving_S = (cur_row == ROW_S && cur_col == COL_S && !is_moving_to_S);
                            bool is_moving_to_special = false;
                            
                            // A, B, C 위치를 제외한 나머지 위치에서 충돌 확인
                            is_moving_to_special = (next_row == robots[i].f_row && next_col == robots[i].f_col);
                            
                            // 이동 충돌 방지 로직
                            if (reserved[next_row][next_col] && !is_moving_to_special) {
                                // 이미 다른 로봇이 예약한 칸이면 대기
                                direction = 4;
                                next_row = cur_row;
                                next_col = cur_col;
                                
                                // 현재 위치 다시 점유 표시
                                reserved[cur_row][cur_col] = 1;
                                
                                // S 위치에서 나가려고 했으나 대기하게 된 경우, S 위치는 계속 점유 상태 유지
                                if (is_leaving_S) {
                                    s_location_occupied = 1;
                                    reserved[ROW_S][COL_S] = 1;
                                }
                            } else {
                                // 다음 위치 예약
                                reserved[next_row][next_col] = 1;
                                
                                // S 위치 특별 처리
                                if (is_moving_to_S) {
                                    s_location_occupied = 1;
                                } else if (is_leaving_S) {
                                    // S 위치에서 나가는 경우 성공적으로 이동
                                    s_location_occupied = 0;
                                    reserved[ROW_S][COL_S] = 0;
                                }
                            }
                        } else {
                            // 제자리 유지의 경우 현재 위치 다시 점유 표시
                            reserved[cur_row][cur_col] = 1;
                        }
                    }
                    
                    // 명령 전달 (다음 좌표 및 방향 포함)
                    message_from_center(i, &boxes_from_central_control_node[i], next_row, next_col);
                    // 메시지 처리 완료
                    boxes_from_robots[i].dirtyBit = 0;
                }
            }
            increase_step();     // 시각화용 step 증가
            unblock_threads();   // 모든 로봇 스레드 깨우기
        }
    }
}


/* 물류 로봇 스레드 */
void robots_thread(void* aux) {
    int idx = *((int *)aux);

    while (1) {
        if (boxes_from_central_control_node[idx].dirtyBit == 1) {
            int next_row = boxes_from_central_control_node[idx].msg.row;
            int next_col = boxes_from_central_control_node[idx].msg.col;

	    robots[idx].row = next_row;
            robots[idx].col = next_col;

            // 센터에 메시지 보내기
            message_from_robots(idx, &boxes_from_robots[idx], robots[idx].row, robots[idx].col);
            boxes_from_central_control_node[idx].dirtyBit = 0;
        }

        block_thread();  // 스스로 블록 상태로
    }
}

// 시뮬레이터 진입점
void run_automated_warehouse(char **argv) {
    init_automated_warehouse(argv);

    printf("자동화 창고 시뮬레이션 시작!\n");

    n = atoi(argv[1]); // 로봇 수 파싱
    char *config_string = NULL;
    if (argv[2] != NULL) {
        config_string = malloc(strlen(argv[2]) + 1);
        strlcpy(config_string, argv[2], strlen(argv[2]) + 1);
    }
    
    // 파싱 (예: "2A:1B:3C" → ["2A", "1B", "3C"])
    char *robot_configs[n + 1];
    char *saveptr;
    char *config_token = config_string ? strtok_r(config_string, ":", &saveptr) : NULL;
    int config_idx = 0;
    while (config_token != NULL && config_idx < n) {
        robot_configs[config_idx++] = config_token;
        config_token = strtok_r(NULL, ":", &saveptr);
    }

    initialize_obstacle_map(); // 이동에 참고할 maps 초기화
    message_boxes(n+1); // 모든 스레드가 참고할 message_box 초기화

    // 로봇 생성
    step = 0;
    robots = malloc(sizeof(struct robot) * (n+1));
    for (int i = 0; i < n; i++) {
        char name[10];
        snprintf(name, sizeof(name), "R%d", i+1);
        NumCharPair robot_config = split_string(robot_configs[i]); // Ex) "2A" => {2, 'A'}

        // Ex) 2와 'A'에 해당하는 좌표 각각 구하기
        Position loading_pos = findPosition(robot_config.num + '0'); // '0' 넣은 이유 : 숫자를 문자로 바꾸기 위해
        Position destination_pos = findPosition(robot_config.letter);
        
        char* robot_name =  create_string_copy(name);
        setRobot(&robots[i], robot_name, ROW_W, COL_W, 1, 0, loading_pos.row, loading_pos.col, destination_pos.row, destination_pos.col, 0);
    }

    // 스레드 생성
    tid_t* threads = malloc(sizeof(tid_t) * (n+1));
    int* idx = malloc(sizeof(int) * (n+1));
    for (int i = 0; i < n; i++) {
        idx[i] = i;
        threads[i] = thread_create(robots[i].name, 0, &robots_thread, &idx[i]);
    }
    
    // 중앙 제어 노드 스레드 생성
    thread_create("CNT", 0, &cnt_thread, NULL);

}
