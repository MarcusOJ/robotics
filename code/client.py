import websocket

ws = websocket.create_connection('ws://localhost:8765')

while True:
    print(ws.recv())