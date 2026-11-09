import matplotlib.pyplot as plt
from ultralytics import YOLO

model = YOLO("runs/segment/train/weights/best.pt")   # your trained weights
r = model.predict("testing/tiles/img1(2,2).png", imgsz=640, conf=0.1)[0]

n = 0 if r.masks is None else len(r.masks)
print(f"{n} rocks found")

plt.imshow(r.plot()[..., ::-1])   # plot() returns BGR → flip to RGB
plt.axis("off")
plt.show()


# path = "testing/tiles/img1(2,0).png"
# for imgsz in (640, 1344):
#     for conf in (0.25, 0.10, 0.05):
#         r = model.predict(path, imgsz=imgsz, conf=conf, verbose=False)[0]
#         n = 0 if r.masks is None else len(r.masks)
#         print(f"imgsz={imgsz:4d}  conf={conf:.2f}  ->  {n} rocks")