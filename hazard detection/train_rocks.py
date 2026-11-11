from ultralytics import YOLO


def main():
    model = YOLO("yolo26n-seg.pt") # using a COCO-pretrained seg model
    model.train(
        data="data/data.yaml",
        epochs=100,
        imgsz=640,
        batch=16,
        patience=20, # stops early if val stops improving for 20 seconds
        device=0
    )
    model.val(split="test")

if __name__ == "__main__":
    main()