# Try outs for pycode
import random

# Start movements
random_movement = random.randrange(0,3)
print(random_movement)


# Move forwards
if random_movement == 1:
    ser.write(1)
    time.sleep(2)

# Move backwards
if random_movement == 0:
    ser.write(2)
    time.sleep(2)

# Move Right
if random_movement == 2:
    ser.write(1)
    time.sleep(2)

# Move Left
if random_movement == 3:
    ser.write(2)
    time.sleep(2)
