import asyncio
import websockets

async def hello(websocket, path):
    
    sdf = input("send: ")
    await websocket.send(sdf)
    print(f"> {sdf}")

start_server = websockets.serve(hello, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()