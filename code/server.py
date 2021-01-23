#!/usr/bin/env python

import asyncio
import websockets
import json


@asyncio.coroutine
async def hello(websocket, uri):
    while True:
        cmd = int(input("vali käsk"))
        if cmd == 1:
            name = {
                "signal": "start",
                "targets": ["Io", "tulihand"],
                "baskets": ["blue", "magenta"]
            }
        elif cmd == 2:
            name = {
                "signal": "stop",
                "targets": ["Io", "tulihand"],
            }
        elif cmd == 3:
            name = {
                "signal": "start",
                "targets": ["Io", "tulihand"],
                "baskets": ["magenta", "blue"]
            }
        y = json.dumps(name)

        await websocket.send(y)

# Below line has the SERVER IP address, not client.
start_server = websockets.serve(hello, 'localhost', 8080)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()