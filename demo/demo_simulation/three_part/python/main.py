from send_command import Msg, RobotSocket
import websocket as ws

def success_callback(msg):
    print(msg.data)
    return

if __name__ == "__main__":
    websocket = ws.WebSocket()
    websocket.connect("ws://127.0.0.1:5867")
    rws = RobotSocket(websocket)
    for i in range(10):
        rws.sendCmd("sim_act -a={0,0} -s=25", success_callback)
    websocket.close()
