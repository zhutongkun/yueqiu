from ultralytics import YOLO

# Load a model
model = YOLO("./runs/detect/fruit_picking/weights/best.pt")  # load a pretrained model (recommended for training)

# Use the model
# model.train(data="coco8.yaml", epochs=3)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
results = model("./my_data_fruit/images/train/image_1690.jpg")  # predict on an image
# path = model.export(format="onnx")  # export the model to ONNX format

# Process results generator
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk