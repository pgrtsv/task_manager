## task_manager

Узел автономного управления дроном. Узел способен решать первую задачу Аэробота - влетать в ближайшее здание, исследовать его, находить объекты интереса и возвращаться на исходную точку, и вторую задачу - исследовать комнаты в заданном QR-кодами порядке, приземляться на правильную платформу.

### Использует сервисы:

- /transform/pose
- /transform/point
- /explorer_manager/start
- /explorer_manager/pause
- /vision/holes/pos_collector/get_nearest
- /fuel/reset

### Использует сервисы actionlib:

- /fast_planner_server

### Публикует топики

- /task_manager/events
- /task_manager/status

### Публикует сервисы:

- /task_manager/start
