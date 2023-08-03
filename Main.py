import _thread

from DrRobot import DrRobot

DrRobot = DrRobot()


def move_cmd():
    for i in range(1,2):
        DrRobot.emergency_stop_release()
        DrRobot.turn_left(200, 1)
        DrRobot.go_forward(200, 1)
        DrRobot.turn_right(200, 1)
        DrRobot.go_backward(200, 1)
        DrRobot.emergency_stop()
    DrRobot.close_connection()

def read_sensors():
    DrRobot.read()


_thread.start_new_thread(move_cmd,() )
_thread.start_new_thread(read_sensors, () )
