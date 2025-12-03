const io = require("socket.io-client");
const socket = io("http://localhost:8080", {
    transports: ['websocket', 'polling'],
    reconnection: true
});

socket.on('connect', () => {
    console.log('✅ Simulator connected:', socket.id);
});

socket.on('disconnect', () => {
    console.log('❌ Simulator disconnected');
});

socket.on('connect_error', (err) => {
    console.error('❌ Connection error:', err);
});

let battery = 100;
let missionPhase = 0;
const missionStatuses = [
  "Standby",
  "Start",
  "Floating Ball Set",
  "Surface Imaging",
  "Underwater Imaging",
  "Finish"
];

// Contoh pengiriman data setiap 2 detik (2000 ms)
setInterval(() => {
  // Simulasi penurunan battery
  battery -= Math.random() * 1.5;
  if (battery < 0) battery = 0;

  // Simulasi perubahan status misi
  missionPhase = (missionPhase + 1) % missionStatuses.length;
  const missionStatus = missionStatuses[missionPhase];

  // Data navigasi dummy
  let lat = -5.3972 + (Math.random() * 0.01 - 0.005);
  let lng = 105.2668 + (Math.random() * 0.01 - 0.005);
  let speed = Math.random() * 2 + 0.5; // m/s
  let sog_knot = (speed * 1.94384).toFixed(2);
  let sog_kmh = (speed * 3.6).toFixed(2);
  let cog = (Math.random() * 360).toFixed(1);
  let pitch = (Math.random() * 20 - 10).toFixed(1);
  let roll = (Math.random() * 20 - 10).toFixed(1);

  // Format waktu dan geotag
  const now = new Date();
  const dayNames = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
  const day = dayNames[now.getDay()];
  const date = now.toLocaleDateString('en-GB'); // DD/MM/YYYY
  const time = now.toLocaleTimeString('en-GB'); // HH:MM:SS
  function decimalToDM(decimal, isLat) {
    const abs = Math.abs(decimal);
    const deg = Math.floor(abs);
    const min = ((abs - deg) * 60).toFixed(4);
    const dir = isLat ? (decimal >= 0 ? 'N' : 'S') : (decimal >= 0 ? 'E' : 'W');
    return `${dir} ${deg}° ${min}'`;
  }
  const latDM = decimalToDM(lat, true);
  const lngDM = decimalToDM(lng, false);

  const data = {
    // Attitude panel dan log table
    Day: day,
    Date: date,
    Time: time,
    Latitude_DD: lat.toFixed(5),
    Longitude_DD: lng.toFixed(5),
    Latitude_DM: latDM,
    Longitude_DM: lngDM,
    SOG_knot: sog_knot,
    SOG_kmh: sog_kmh,
    COG_deg: cog,
    Pitch: pitch,
    Roll: roll,
    // Fitur baru!
    Battery_: battery.toFixed(1),
    Mission_Status: missionStatus,
    // Tambahan: url video/foto dummy jika mau tes media
    // videoUrl: "video-url.mp4",
    // photo4Url: "foto4.jpg",
    // photo5Url: "foto5.jpg"
  };

  socket.emit('sensor_data', data);
  // Untuk debug
  console.log("Simulasi data dikirim:", data);
}, 2000);