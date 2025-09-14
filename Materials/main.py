import numpy as np
from time import sleep, perf_counter
from sys import stderr

period = 1/60 # 60 Hz

print(f'''
GYROFLOW IMU LOG
version,1.3
id,custom_logger_name
orientation,YxZ
note,development_test
fwversion,FIRMWARE_0.1.0
timestamp,1644159993
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
'''.strip())

rng = np.random.default_rng()

def _advance_vector(x, v, aabs, *, rho=0.92, dt=0.01):
  sigma_eps = aabs * np.sqrt(1 - rho**2) * np.sqrt(np.pi / 2)
  eps = rng.normal(0.0, sigma_eps, size=aabs.shape)
  v_next = rho * v + eps
  x_next = x + v_next * dt
  return x_next, v_next

def sensors_data_generator():
  aabs = np.array([11.333333, 5.133333, 17.133333, 53.066667, 15.266667, 69.8], dtype=float)
  v = aabs.copy()
  x = np.array([17.0, 14.0, 19.0, -42.0, -5.0, 99.0], dtype=float)
  i = 0
  while True:
    x, v = _advance_vector(x, v, aabs, rho=0.92, dt=0.01)
    print(f"{i}," + ",".join(f"{val:.0f}" for val in x))
    i += 1
    yield

gen = sensors_data_generator()

next_t = perf_counter()
while True:
  start = next_t =  perf_counter()
  next(gen)
  next_t += period
  sleep_time = next_t - perf_counter()
  if sleep_time > 0:
    sleep(sleep_time)
  else:
    print(f"Warning: Overrun detected. Consider adjusting the period.", file=stderr)
    next_t = perf_counter()
