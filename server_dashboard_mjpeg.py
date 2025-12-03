"""
Flask server with MJPEG streaming support
"""
from flask import Flask, send_from_directory, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import threading
import queue
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'saibatin-azura-secret'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Queue untuk frame MJPEG dari Raspberry Pi
frame_queue = queue.Queue(maxsize=2)

@app.route('/')
def index():
    return send_from_directory('.', 'index_mjpeg.html')

@app.route('/index_mjpeg.html')
def index_mjpeg():
    return send_from_directory('.', 'index_mjpeg.html')

def generate_mjpeg():
    """Generator untuk MJPEG stream"""
    while True:
        try:
            # Ambil frame dari queue (timeout 1 detik)
            frame_jpeg = frame_queue.get(timeout=1.0)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_jpeg + b'\r\n')
        except queue.Empty:
            # No frame available, send placeholder or continue
            time.sleep(0.1)
            continue

@app.route('/video_feed')
def video_feed():
    """MJPEG streaming route"""
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    print('✅ Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('❌ Client disconnected')

@socketio.on('update_dashboard')
def handle_dashboard_update(data):
    """GPS/telemetry data"""
    print(f'📊 GPS: Lat={data.get("Latitude_DD")}, Status={data.get("Mission_Status")}')
    emit('update_dashboard', data, broadcast=True)

@socketio.on('camera_frame')
def handle_camera_frame(data):
    """Metadata saja (ball detection, midpoint)"""
    # Jangan kirim image! Image lewat MJPEG
    metadata = {
        'red_ball': data.get('red_ball'),
        'green_ball': data.get('green_ball'),
        'midpoint': data.get('midpoint'),
        'heading_adjustment': data.get('heading_adjustment'),
        'timestamp': time.time()
    }
    emit('camera_frame', metadata, broadcast=True)

@socketio.on('mjpeg_frame')
def handle_mjpeg_frame(data):
    """Terima JPEG binary dari Raspberry Pi"""
    try:
        # data['jpeg'] harus berupa bytes
        jpeg_bytes = data.get('jpeg')
        if jpeg_bytes and isinstance(jpeg_bytes, bytes):
            # Clear queue jika penuh (drop old frame)
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put(jpeg_bytes)
    except Exception as e:
        print(f"⚠️ MJPEG frame error: {e}")

@socketio.on('surface_photo')
def handle_surface_photo(data):
    emit('surface_photo', data, broadcast=True)

@socketio.on('underwater_photo')
def handle_underwater_photo(data):
    emit('underwater_photo', data, broadcast=True)

@socketio.on('mission_checkpoint')
def handle_mission_checkpoint(data):
    emit('mission_checkpoint', data, broadcast=True)

if __name__ == '__main__':
    print("=" * 70)
    print("🚀 SAIBATIN AZURA 1.0 - MJPEG Server")
    print("=" * 70)
    print("📡 Server: http://0.0.0.0:5000")
    print("🌐 Dashboard: http://localhost:5000/index_mjpeg.html")
    print("✅ MJPEG stream: http://localhost:5000/video_feed")
    print("=" * 70)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
