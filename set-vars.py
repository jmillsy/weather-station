# https://blog.yavilevich.com/2020/09/convention-for-compile-time-configuration-of-platformio-projects/

from os.path import isfile
Import("env")
assert isfile(".env")

try:
  with open(".env", "r") as f:
    lines = f.readlines()
    envs = []
    for line in lines:
        stripped = line.strip()
        if stripped and not stripped.startswith('#'):
            # Split key=value and quote the value part
            if '=' in stripped:
                key, value = stripped.split('=', 1)
                envs.append('-D{}="{}"'.format(key, value))
            else:
                envs.append("-D{}".format(stripped))
    env.Append(BUILD_FLAGS=envs)
    print("set-vars.py: Added {} build flags from .env".format(len(envs)))
except IOError:
  print("set-vars.py: File .env not accessible")