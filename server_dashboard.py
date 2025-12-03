"""
Socket.IO Server untuk Dashboard SAIBATIN AZURA 1.0
Support MJPEG streaming untuk efisiensi bandwidth
"""

from flask import Flask, send_from_directory, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import queue
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = 'saibatin-azura-secret'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Queue untuk MJPEG frames
frame_queue = queue.Queue(maxsize=2)
latest_metadata = {}

# Serve index.html
@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/index.html')
def index_html():
    return send_from_directory('.', 'index.html')

def generate_mjpeg():
    """Generator untuk MJPEG stream"""
    while True:
        try:
            frame_jpeg = frame_queue.get(timeout=1.0)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_jpeg + b'\r\n')
        except queue.Empty:
            continue

@app.route('/video_feed')
def video_feed():
    """MJPEG streaming endpoint"""
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
    """GPS/Telemetry data"""
    print(f'📊 GPS: Time={data.get("Time")}, Status={data.get("Mission_Status")}')
    emit('update_dashboard', data, broadcast=True)

@socketio.on('mjpeg_frame')
def handle_mjpeg_frame(data):
    """Terima JPEG bytes dari Raspberry Pi untuk MJPEG stream"""
    try:
        jpeg_bytes = data.get('jpeg')
        if jpeg_bytes and isinstance(jpeg_bytes, bytes):
            # Clear old frame jika queue penuh
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put(jpeg_bytes)
    except Exception as e:
        print(f"⚠️ MJPEG frame error: {e}")

@socketio.on('camera_frame')
def handle_camera_frame(data):
    """Metadata saja (ball detection info)"""
    # Tidak kirim image! Hanya metadata
    metadata = {
        'red_ball': data.get('red_ball'),
        'green_ball': data.get('green_ball'),
        'midpoint': data.get('midpoint'),
        'heading_adjustment': data.get('heading_adjustment'),
        'direction': data.get('direction'),
        'balls_passed': data.get('balls_passed')
    }
    emit('camera_frame', metadata, broadcast=True)
    print(f'📷 Metadata: Red={bool(metadata["red_ball"])}, Green={bool(metadata["green_ball"])}')

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
    print("🚀 SAIBATIN AZURA 1.0 Socket.IO Server (MJPEG)")
    print("=" * 70)
    print("📡 Server running on: http://0.0.0.0:5000")
    print("🌐 Open dashboard: http://localhost:5000/index.html")
    print("🎥 MJPEG stream: http://localhost:5000/video_feed")
    print("✅ Ready to receive data from Raspberry Pi")
    print("=" * 70)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
