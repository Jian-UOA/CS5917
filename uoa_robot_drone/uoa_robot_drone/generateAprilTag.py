import numpy as np
from PIL import Image

# 这是 tag36h11 family, id=0 的二进制编码（5x5内码，带边框7x7）
tag_bits = [
    [1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1],
    [1,0,1,1,0,1,1],
    [1,0,1,0,1,0,1],
    [1,0,0,1,1,1,1],
    [1,0,1,0,0,0,1],
    [1,1,1,1,1,1,1]
]
tag = np.array(tag_bits, dtype=np.uint8) * 255
img = Image.fromarray(tag).resize((350,350), resample=Image.NEAREST)
img.save('apriltag36h11_id0.png')
img.show()