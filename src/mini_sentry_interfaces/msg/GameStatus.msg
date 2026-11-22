# 比赛状态（由 referee_sim 模拟裁判系统发布）
uint8 PROGRESS_NOT_STARTED=0
uint8 PROGRESS_PREPARING=1
uint8 PROGRESS_RUNNING=4
uint8 PROGRESS_FINISHED=5

uint8  game_progress
uint16 stage_remain_time      # 本局剩余秒数
uint16 current_hp
uint16 maximum_hp
