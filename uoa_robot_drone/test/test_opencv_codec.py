
# 本地编码器支持性测试脚本
# 生成假帧，循环写入视频文件，测试不同编码器
import time, cv2
import numpy as np

height, width = 720, 960
fps = 30
duration = 3  # 秒
num_frames = fps * duration

# 你可以切换下面的fourcc和文件名，测试不同编码器
codecs = [
    ("XVID", "test_xvid.avi"),
    ("MJPG", "test_mjpg.avi"),
    ("mp4v", "test_mp4v.mp4"),
]

for fourcc_str, filename in codecs:
    print(f"\nTesting codec: {fourcc_str}, output: {filename}")
    fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
    video = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    if not video.isOpened():
        print(f"Failed to open VideoWriter for {filename} with codec {fourcc_str}")
        continue
    for i in range(num_frames):
        # 生成一张彩色渐变假帧
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        color = np.array([(i*5) % 255, (i*3) % 255, (i*7) % 255], dtype=np.uint8)
        frame[:] = color
        cv2.putText(frame, f"Frame {i+1}", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 3)
        video.write(frame)
        cv2.imshow("test", frame)
        cv2.waitKey(1)
        time.sleep(1/fps)
    video.release()
    print(f"Finished writing {filename}")

cv2.destroyAllWindows()
print("All codec tests done. 请检查生成的视频文件能否正常播放，有无警告或报错。")
