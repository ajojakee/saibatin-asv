const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const fs = require('fs');
const path = require('path');
const request = require('request'); // Tambahan untuk MJPEG proxy

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"]
  },
  pingTimeout: 60000,      // Increase ping timeout
  pingInterval: 25000,     // Increase ping interval
  upgradeTimeout: 30000,   // Increase upgrade timeout
  allowEIO3: true          // Allow older engine.io clients
});

// --- Logging ke CSV ---
const LOG_FILE = path.join(__dirname, 'log-geotag.csv');
if (!fs.existsSync(LOG_FILE)) {
  const headers = 'Timestamp,Day,Date,Time,Latitude_DD,Longitude_DD,Latitude_DM,Longitude_DM,SOG_knot,SOG_kmh,COG_deg,Battery_%,Mission_Status,Pitch,Roll\n';
  fs.writeFileSync(LOG_FILE, headers);
}

function logToCsv(data) {
  const timestamp = new Date().toISOString();
  const row = `${timestamp},${data.Day},${data.Date},${data.Time},${data.Latitude_DD},${data.Longitude_DD},${data.Latitude_DM},${data.Longitude_DM},${data.SOG_knot},${data.SOG_kmh},${data.COG_deg},${data.Battery_},${data.Mission_Status},${data.Pitch},${data.Roll}\n`;
  fs.appendFileSync(LOG_FILE, row, 'utf8');
}

// Serve static files
app.use(express.static(__dirname));

// Proxy MJPEG endpoint Pi ke dashboard web
app.get('/video_feed', (req, res) => {
  // Ubah MJPEGURL sesuai IP/port Raspberry Pi yang serve MJPEG!
  const MJPEGURL = 'http://10.192.7.110:5000/video_feed';
  req.setTimeout(0);
  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  request(MJPEGURL)
    .on('error', (err) => {
      console.error('❌ Error MJPEG proxy:', err.message);
      res.status(502).send('MJPEG stream unavailable');
    })
    .pipe(res);
});

app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// Real-time Socket.IO events
io.on('connection', (socket) => {
    console.log('✅ Client connected:', socket.id);
    console.log('   Transport:', socket.conn.transport.name);
    console.log('   Address:', socket.handshake.address);
    console.log('📊 Total clients:', io.engine.clientsCount);

    // Keep connection alive
    const keepAlive = setInterval(() => {
        socket.emit('ping', { timestamp: Date.now() });
    }, 20000);

    // Handler untuk sensor_data (simulator lama)
    socket.on('sensor_data', (data) => {
        console.log('📡 sensor_data from:', socket.id);
        try {
            logToCsv(data);
            console.log('✅ Data logged to CSV');
        } catch (err) {
            console.error('❌ Error logging to CSV:', err);
        }
        io.emit('update_dashboard', data);
        console.log('📤 Broadcasted sensor_data as update_dashboard');
    });

    // Handler untuk update_dashboard dari Raspberry Pi
    socket.on('update_dashboard', (data) => {
        console.log('📡 update_dashboard from:', socket.id);
        try {
            logToCsv(data);
        } catch (err) {
            console.error('❌ Error logging to CSV:', err);
        }
        io.emit('update_dashboard', data);
        console.log('📤 Broadcasted update_dashboard to', io.engine.clientsCount, 'clients');
    });

    // Handler untuk video_frame dari Raspberry Pi (CRITICAL!)
    socket.on('video_frame', (data) => {
        // JANGAN LOG DETAIL (terlalu banyak), cukup count
        io.emit('video_frame', data);
    });

    // Handler untuk photo_captured
    socket.on('photo_captured', (data) => {
        console.log('📸 photo_captured:', data.type, 'gate:', data.gate);
        io.emit('photo_captured', data);
    });

    // Handler untuk gate_passed
    socket.on('gate_passed', (data) => {
        console.log('🚪 gate_passed:', data.gate_type);
        io.emit('gate_passed', data);
    });

    // Handler untuk obstacles_detected
    socket.on('obstacles_detected', (data) => {
        console.log('🚧 obstacles_detected:', data.count, 'obstacles');
        io.emit('obstacles_detected', data);
    });

    // Handler untuk mission_event
    socket.on('mission_event', (data) => {
        console.log('🎯 mission_event:', data.event);
        io.emit('mission_event', data);
    });

    // Handler untuk mission_status_update
    socket.on('mission_status_update', (data) => {
        console.log('🚀 mission_status_update:', data.state);
        io.emit('mission_status_update', data);
    });

    socket.on('disconnect', (reason) => {
        clearInterval(keepAlive);
        console.log('❌ Client disconnected:', socket.id, 'Reason:', reason);
        console.log('📊 Remaining clients:', io.engine.clientsCount);
    });

    socket.on('error', (error) => {
        console.error('❌ Socket error:', error);
    });
});

const PORT = process.env.PORT || 5000;
server.listen(PORT, '0.0.0.0', () => {
  console.log('========================================');
  console.log(`🚀 Server running on http://localhost:${PORT}`);
  console.log('========================================');
});