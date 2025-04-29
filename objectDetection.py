from ultralytics import YOLO
import cv2
import numpy as np

# Load YOLOv8 model (use your custom model if needed)
model = YOLO("yolov8n.pt")  # Replace with "best.pt" if trained

# COCO class IDs
PERSON_CLASS_ID = 0
FURNITURE_CLASSES = {
    56: "chair",
    57: "couch",
    60: "bed",
    61: "dining table",
    62: "toilet",
    69: "oven",
    71: "sink",
    72: "refrigerator"
}

def detect_people(image: np.ndarray) -> int:
    """
    Detects people in an image using YOLOv8.

    Args:
        image (np.ndarray): Input image (BGR format).

    Returns:
        int: Number of people detected.
    """
    if image is None:
        return 0

    results = model(image, verbose=False)
    people_count = 0

    for r in results:
        if r.boxes is not None:
            classes = r.boxes.cls.cpu().numpy().astype(int)
            people_count += np.sum(classes == PERSON_CLASS_ID)

    return people_count

def detect_furniture(image: np.ndarray):
    if image is None:
        return []

    results = model(image, verbose=False)
    obstacles = []

    for r in results:
        if r.boxes is not None:
            boxes = r.boxes
            for i in range(len(boxes.cls)):
                cls_id = int(boxes.cls[i].item())
                if cls_id in FURNITURE_CLASSES:
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                    box_height = y2 - y1
                    box_width = x2 - x1
                    aspect_ratio = box_width / (box_height + 1e-5)

                    # 🚫 Ignore horizontal railing-like boxes
                    image_mid_y = image.shape[0] // 2
                    if box_height < 80 and aspect_ratio > 2.5 and y2 < image_mid_y:
                        continue  # likely a railing


                    x_center = (x1 + x2) / 2
                    obstacles.append({
                        'class_id': cls_id,
                        'label': FURNITURE_CLASSES[cls_id],
                        'x_center': float(x_center),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)]
                    })

    return obstacles



def detect_back_wall_or_door(image: np.ndarray):
    results = model(image, verbose=False)
    if image is None or results is None:
        return None

    width = image.shape[1]
    farthest_y = 0

    for r in results:
        if r.boxes is not None:
            for box in r.boxes.xyxy:
                _, y1, _, y2 = map(int, box.cpu().numpy())
                y_center = (y1 + y2) / 2
                if y_center > farthest_y:
                    farthest_y = y_center

    return farthest_y  # pixel location of the farthest detected object


def detect_clear_floor_zones(image: np.ndarray) -> list:
    """
    Detect regions on the floor free from obstacles.

    Args:
        image (np.ndarray): Input image.

    Returns:
        List of dictionaries with x_center and y_center of clear zones.
    """
    results = model(image, verbose=False)
    height, width = image.shape[:2]

    # Initialize a mask of all zeros
    mask = np.zeros((height, width), dtype=np.uint8)

    for r in results:
        if r.boxes is not None:
            for box in r.boxes.xyxy:
                x1, y1, x2, y2 = map(int, box.cpu().numpy())
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)  # mark obstacle regions

    # Invert the mask to get clear regions
    clear_mask = cv2.bitwise_not(mask)
    contours, _ = cv2.findContours(clear_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    zones = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 5000:  # only large clear zones
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                zones.append({'x_center': cx, 'y_center': cy})
    return zones


# if __name__ == "__main__":
#     # Quick test
#     img = cv2.imread("test_image.jpg")
#     people = detect_people(img)
#     furniture = detect_furniture(img)

#     print(f"People detected: {people}")
#     for obj in furniture:
#         print(f"Furniture: {obj['label']} at x={obj['x_center']:.1f}")
