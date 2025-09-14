import asyncio
import websockets
import numpy as np
from time import perf_counter, time as now
from sys import stderr

period = 1/60  # 60 Hz

header = f'''
GYROFLOW IMU LOG
version,1.3
id,custom_logger_name
orientation,YxZ
note,development_test
fwversion,FIRMWARE_0.1.0
timestamp,{now()+50}
vendor,potatocam
videofilename,videofilename.mp4
lensprofile,potatocam/potatocam_mark1_prime_7_5mm_4k
lens_info,wide
frame_readout_time,15.23
frame_readout_direction,0
tscale,{period}
gscale,0.00122173047
ascale,0.00048828125
t,gx,gy,gz,ax,ay,az
'''.strip()

rng = np.random.default_rng()

def _advance_vector(x, v, aabs, *, rho=0.92, dt=0.01):
    sigma_eps = aabs * np.sqrt(1 - rho**2) * np.sqrt(np.pi / 2)
    eps = rng.normal(0.0, sigma_eps, size=aabs.shape)
    v_next = rho * v + eps
    x_next = x + v_next * dt
    return x_next, v_next

def sensors_data_generator():
    """Yields raw sensor values only (without counter)."""
    aabs = np.array([11.333333, 5.133333, 17.133333, 53.066667, 15.266667, 69.8], dtype=float)
    aabs *= 200
    v = aabs.copy()
    x = np.array([17.0, 14.0, 19.0, -42.0, -5.0, 99.0], dtype=float)
    while True:
        x, v = _advance_vector(x, v, aabs, rho=0.92, dt=0.01)
        yield ",".join(f"{val:.0f}" for val in x)

# --- WebSocket server ---
clients = {}  # maps websocket -> counter

async def register_client(ws):
    clients[ws] = 0  # start counter at 0 for this client
    print(f"[+] Client connected ({len(clients)} total)")
    try:
        await ws.send(header)  # send header immediately
        await ws.wait_closed()
    finally:
        clients.pop(ws, None)
        print(f"[-] Client disconnected ({len(clients)} total)")

async def broadcast_data():
    gen = sensors_data_generator()
    next_t = perf_counter()
    while True:
        values = next(gen)  # raw sensor values (no counter)

        if clients:
            to_remove = []
            for ws, counter in list(clients.items()):
                try:
                    line = f"{counter},{values}"
                    await ws.send(line)
                    clients[ws] = counter + 1  # increment per-client counter
                except Exception:
                    to_remove.append(ws)
            for ws in to_remove:
                clients.pop(ws, None)
                print("[-] Removed closed client")

        next_t += period
        sleep_time = next_t - perf_counter()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            print("Warning: Overrun detected.", file=stderr)
            next_t = perf_counter()

async def main():
    async with websockets.serve(register_client, "0.0.0.0", 5000):
        print("[*] WebSocket server running on ws://0.0.0.0:5000")
        await broadcast_data()

if __name__ == "__main__":
    asyncio.run(main())
