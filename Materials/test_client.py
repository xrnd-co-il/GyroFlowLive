import asyncio
import websockets

async def client():
    uri = "ws://127.0.0.1:5000"
    async with websockets.connect(uri) as ws:
        try:
            async for msg in ws:
                print(msg)
        except asyncio.CancelledError:
            pass

asyncio.run(client())
