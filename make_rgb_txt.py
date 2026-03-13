import os

img_dir = "data/extracted_data"
images = sorted([f for f in os.listdir(img_dir) if f.endswith('.jpg')])

with open(os.path.join(img_dir, "rgb.txt"), "w") as f:
    for img_name in images:
        # 提取文件名中的时间戳部分（去掉 .jpg）
        timestamp = img_name.replace('.jpg', '')
        f.write(f"{timestamp} {img_name}\n")

print("Done! rgb.txt is now perfectly synced with real timestamps.")