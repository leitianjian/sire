import send_command

def success_callback(msg):
    print(msg.data)
    return

if __name__ == "__main__":
    ws = send_command.RobotSocket()
    ws.sendCmd("sim_act -a={0,0} -s=25", success_callback)
