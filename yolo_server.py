from ultralytics import YOLO
from flask import Flask, request, jsonify
import cv2
import numpy as np
import base64
import datetime

# Инициализация Flask и YOLO
app = Flask(__name__)
model = YOLO("yolov8n.pt") # Базовая модель

@app.route('/detect', methods=['POST'])
def detect():
    try:
        # Получаем изображение от дрона
        data = request.json
        img_data = base64.b64decode(data['image'])
        nparr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Прогоняем через YOLO
        results = model(img)

        # Формируем ответ
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                detections.append({
                    "class": model.names[cls],
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })

        # 🟢 ВЫВОДИМ В ТЕРМИНАЛ ТО, ЧТО НАШЕЛ YOLO
        if detections:
            current_time = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"[{current_time}] 🎯 YOLO нашел: {detections[0]['class']} (уверенность: {detections[0]['confidence']:.2f})")
            # Если объектов несколько, выведем первый
        else:
            print("👀 Ничего не найдено в этом кадре.")

        return jsonify({"detections": detections})

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    print("🚀 YOLO Server запущен на порту 5000")
    app.run(host='0.0.0.0', port=5000)
