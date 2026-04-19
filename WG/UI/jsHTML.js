let frontendReady = false;
let pendingQueue = [];

var map;        // 全域地圖物件
var polyline;   // 預先宣告，初始化放在 onload 裡
var marker = null;
var lastLatLng = null;
var trajectory = [];
let polylines_current = [];
let polylines_dubins = [];


let targetLatLng = null;
let targetSelectionEnabled = false;
let targetMarker = null;

let tileCount = 0;

const myIcon = L.icon({
  iconUrl: 'images/images/marker-icon.png',
  shadowUrl: 'images/images/marker-shadow.png',
  iconSize:     [25, 41],
  iconAnchor:   [12, 41],
  popupAnchor:  [1, -34],
  shadowSize:   [41, 41]
});




document.addEventListener("DOMContentLoaded", () => {
  setTimeout(() => {
    initMap();
  }, 100);

  const pidBtn = document.getElementById('pid-config-btn');
  if (!pidBtn) {
    console.warn("找不到 #pid-config-btn");
    return;
  }
  pidBtn.addEventListener('click', () => {
    window.pywebview.api.open_pid();
  });

  setTimeout(() => {
    frontendReady = true;
    console.log("✅ UI ready, flushing pending queue");

    while (pendingQueue.length > 0) {
      const task = pendingQueue.shift();
      try {
        task();
      } catch (e) {
        console.error("❌ 錯誤執行任務：", e);
      }
    }
        // 通知 Python
    if (window.pywebview?.api?.notify_ready) {
      window.pywebview.api.notify_ready();
    }
  }, 500);  // ⏱ 建議設個 300～1000ms 確保全部元素都掛好
});


// function initMap() {
//   map = L.map('map').setView([25.15, 121.75], 13);

//   // ✅ 使用本地圖磚資料夾 tiles/
//   const offlineLayer = L.tileLayer('tiles/{z}/{x}/{y}.png', {
//     attribution: '© OpenStreetMap contributors',
//     minZoom: 10,
//     maxZoom: 23,
//     errorTileUrl: 'tiles/empty.png' // 預設空白圖（可選）
//   });

//   offlineLayer.addTo(map);

//   setTimeout(() => {
//     map.invalidateSize();
//     console.log("✅ Leaflet map size fixed:", map.getSize());
//   }, 100);

//   polyline = L.polyline([], { color: 'red' }).addTo(map);

//   map.on("click", function (e) {
//     if (!targetSelectionEnabled) return;

//     const clickedLat = e.latlng.lat;
//     const clickedLon = e.latlng.lng;
//     const clickedPoint = L.latLng(clickedLat, clickedLon);

//     targetLatLng = clickedPoint;

//     if (targetMarker) map.removeLayer(targetMarker);
//     targetMarker = L.marker(clickedPoint, { icon: myIcon }).addTo(map);

//     if (lastLatLng) {
//       const distance = clickedPoint.distanceTo(lastLatLng);
//       document.getElementById("latlon").innerText =
//         `與 WG 距離：${distance.toFixed(2)} 公尺`;
//     } else {
//       document.getElementById("latlon").innerText = `尚未獲得 Wave Glider 位置`;
//     }
//   });

//   setInterval(() => {
//     window.pywebview.api.check_socket().then((connected) => {
//       updateSocketIndicator(connected);
//     });

//     if (lastLatLng && targetLatLng) {
//       const distance = lastLatLng.distanceTo(targetLatLng);
//       document.getElementById("latlon").innerText =
//         `與 WG 距離：${distance.toFixed(2)} 公尺`;
//     }
//   }, 3000);
// }

function initMap() {
  // 1. 建立地圖並設定初始視角
  map = L.map('map').setView([25.15, 121.75], 13);

  // 2. 加入本地離線圖磚
  const offlineLayer = L.tileLayer('tiles/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors',
    minZoom: 10,
    maxZoom: 23,
    errorTileUrl: 'tiles/empty.png'
  });
  offlineLayer.addTo(map);

  // 3. 修正地圖大小
  setTimeout(() => {
    map.invalidateSize();
    console.log("✅ Leaflet map size fixed:", map.getSize());
  }, 100);

  // 4. 建立航跡 polyline
  polyline = L.polyline([], { color: 'red' }).addTo(map);

 

  // 6. 點擊地圖選目標點（與之前相同）
  map.on("click", function (e) {
    if (!targetSelectionEnabled) return;

    const clickedPoint = e.latlng;
    targetLatLng = clickedPoint;

    if (targetMarker) map.removeLayer(targetMarker);
    targetMarker = L.marker(clickedPoint, { icon: myIcon }).addTo(map);

    if (lastLatLng) {
      const distance = clickedPoint.distanceTo(lastLatLng);
      document.getElementById("latlon").innerText =
        `與目標距離：${distance.toFixed(2)} 公尺`;
    } else {
      document.getElementById("latlon").innerText = `尚未獲得 Wave Glider 位置`;
    }
  });

  // 7. 定時檢查連線並更新距離
  setInterval(() => {
    window.pywebview.api.check_socket().then((connected) => {
      updateSocketIndicator(connected);
    });

    if (lastLatLng && targetLatLng) {
      const distance = lastLatLng.distanceTo(targetLatLng);
      document.getElementById("latlon").innerText =
        `與目標距離：${distance.toFixed(2)} 公尺`;
    }
  }, 3000);
}

function drawDubinsPathTrajectory(points) {
  if (!frontendReady) {
    pendingQueue.push(() => drawDubinsPathTrajectory(points));
    return;
  }

  // 移除舊的 dubins path
  for (let line of polylines_dubins) {
    map.removeLayer(line);
  }
  polylines_dubins = [];

  // 建立灰色透明路徑
  const path = points.map(p => L.latLng(p[0], p[1]));
  const newLine = L.polyline(path, { 
    color: "gray", 
    weight: 3, 
    opacity: 0.5 
  }).addTo(map);

  polylines_dubins.push(newLine);

  console.log("✅ 繪製 Dubins path，共", points.length, "點");
}



function drawCurrentPathTrajectory(points, type = "blue") {

  if (!frontendReady) {
    pendingQueue.push(() => drawCurrentPathTrajectory(points, type = "blue"));
    return;
  }

  for (let line of polylines_current) {
    map.removeLayer(line);
  }
  polylines_current = [];

  const color = type === "blue" ? "blue" : "red";
  const path = points.map(p => L.latLng(p[0], p[1]));
  const newLine = L.polyline(path, { color: color }).addTo(map);
  polylines_current.push(newLine);

  console.log("✅ 繪製軌跡，共", points.length, "點");
}

let originMarkers = [];
let arrowLine = null;
let arrowHead = null;

// =====================
// 🎯 畫原點

function drawOrigin(points) {
  if (!frontendReady) {
    pendingQueue.push(() => drawOrigin(points));
    return;
  }

  // 先清掉舊的
  originMarkers.forEach(m => map.removeLayer(m));
  originMarkers = [];
  // points 應該是一個 [[lat, lon], [lat, lon], ...] 的陣列
  for (const [lat, lon] of points) {
    const marker = L.circleMarker([lat, lon], {
      radius: 6,
      color: "black",
      fillColor: "yellow",
      fillOpacity: 1
    }).addTo(map);

    originMarkers.push(marker);
  }

  console.log("✅ 畫出原點數量:", points.length);
}


// ➡️ 畫箭頭
function drawArrow(data) {
  const [lat, lon, angleDeg, length] = data
  if (!frontendReady) {
    pendingQueue.push(() => drawArrow(lat, lon, angleDeg, length));
    return;
  }

  // 移除舊的
  if (arrowLine) map.removeLayer(arrowLine);
  if (arrowHead) map.removeLayer(arrowHead);

  // 角度轉弧度（0° = 北, 90° = 東）
  const angleRad = (angleDeg * Math.PI) / 180.0;

  // 算終點 (平面近似)
  const dLat = (length * Math.cos(angleRad)) / 111320;
  const dLon = (length * Math.sin(angleRad)) / (111320 * Math.cos(lat * Math.PI / 180));
  const endLat = lat + dLat;
  const endLon = lon + dLon;

  // 畫線
  arrowLine = L.polyline([[lat, lon], [endLat, endLon]], {
    color: "purple",
    weight: 3,
    opacity: 0.8
  }).addTo(map);

  // 在終點加箭頭 (SVG + rotate)
  arrowHead = L.marker([endLat, endLon], {
    icon: L.divIcon({
      className: "",
      html: `
        <svg width="20" height="20" viewBox="0 0 100 100" 
             style="transform: rotate(${angleDeg}deg)">
          <polygon points="10,50 90,20 90,80" fill="purple"/>
        </svg>
      `,
      iconSize: [20, 20],
      iconAnchor: [10, 10]
    })
  }).addTo(map);

  console.log("✅ 畫箭頭", { start: [lat, lon], end: [endLat, endLon], angleDeg, length });
}


// =====================

let simulatePolyline = null;

function drawSimulate(points) {
  if (!frontendReady) {
    pendingQueue.push(() => drawSimulate(points));
    return;
  }

  if (simulatePolyline) {
    map.removeLayer(simulatePolyline);
  }

  const path = points.map(p => L.latLng(p[0], p[1]));
  simulatePolyline = L.polyline(path, { color: "green" }).addTo(map);
}



function addPoint(lat, lon) {

  if (!frontendReady) {
    pendingQueue.push(() => addPoint(lat, lon));
    return;
  }

  var newLatLng = L.latLng(lat, lon);
  trajectory.push(newLatLng);
  polyline.setLatLngs(trajectory);
  if (marker) map.removeLayer(marker);
  marker = L.marker(newLatLng, { icon: myIcon }).addTo(map);
  
  lastLatLng = newLatLng;
}

// function 

function Arrive() {
  alert('到達目標')
}

function enableTargetSelection() {
  targetSelectionEnabled = true;
  alert(" 請點擊地圖以選擇目標點");
}

function confirmTarget() {
  if (!targetLatLng) {
    alert("⚠️ 請先點選地圖選擇目標點！");
    return;
  }

  const lat = targetLatLng.lat.toFixed(8);
  const lon = targetLatLng.lng.toFixed(8);

  if (window.pywebview && window.pywebview.api) {
    const cmd = 'setGoal';
    const jsonData = JSON.stringify({ data: targetLatLng });
    const message = `$${cmd}@${jsonData}*\n`;
    window.pywebview.api.send_cmd(message);
    alert(` 目標點已送出：Lat ${lat}, Lon ${lon}`);
  }

  targetSelectionEnabled = false;
  map.getContainer().style.cursor = "";
  console.log(" 已退出選點模式");
}

function zoomToCurrent() {
  if (lastLatLng) {
    map.setView(lastLatLng, 18);
  }
}


function clearTrajectory() {
  trajectory = [];
  polyline.setLatLngs([]);
  if (marker) {
    map.removeLayer(marker);
    marker = null;
  }
  lastLatLng = null;
  document.getElementById('latlon').innerText = '目前位置：無';
}

function updateSocketIndicator(connected) {
  const indicator = document.getElementById("socket-indicator");
  if (connected) {
    indicator.classList.add("connected");
  } else {
    indicator.classList.remove("connected");
  }
}

function propeller(data)
{
  const ind = document.getElementById('prop-indicator');
  if (data) ind.classList.add('connected');
  else       ind.classList.remove('connected');
}

function navigation(data)
{

  const ind = document.getElementById('nav-indicator');
  if (data) ind.classList.add('connected');
  else       ind.classList.remove('connected');
}

function experimentStart() {
  if (window.pywebview && window.pywebview.api) {
    const cmd = 'experimentStart';
    const jsonData = JSON.stringify({ data: 1 });
    const message = `$${cmd}@${jsonData}*\n`;
    window.pywebview.api.send_cmd(message);
    alert('實驗開始');
  }
}

function experimentFinish() {
  if (window.pywebview && window.pywebview.api) {
    const cmd = 'experimentFinish';
    const jsonData = JSON.stringify({ data: 1 });
    const message = `$${cmd}@${jsonData}*\n`;
    window.pywebview.api.send_cmd(message);
    alert('實驗結束');
  }
}

function noGoal() {
  alert('請設定座標');
}


function updateStats(stats) {
  document.getElementById('wgSpeed').textContent   = stats.wgSpeed.toFixed(2);
  document.getElementById('flowSpeed').textContent = stats.flowSpeed.toFixed(2);
  document.getElementById('flowDir').textContent   = stats.flowDir.toFixed(1);

  // ❶ 更新「實際航向（紅色箭頭）」：
  const course = stats.heading || 0;
  
  const arrowGroup = document.getElementById('arrow-group');
  if (arrowGroup) {
    arrowGroup.setAttribute('transform', `rotate(${course}, 50, 50)`);
  }

  // ❷ 更新「期望航向（藍色箭頭）」：
  const exceptCourse = stats.exceptCourse || 0;
  const targetGroup = document.getElementById('target_arrow_group');
  if (targetGroup) {
    targetGroup.setAttribute('transform', `rotate(${exceptCourse}, 50, 50)`);
  }

  const rudderAngle = stats.rudderAngle || 0;
  const rudderGroup = document.getElementById('rudder_arrow_group');
  if (rudderGroup) {
    rudderGroup.setAttribute('transform', `rotate(${-rudderAngle+180}, 50, 50)`); 
  }
  // ❸ 更新下方顯示文字
  document.getElementById('courseValue').textContent       = `${course.toFixed(1)}°`;
  document.getElementById('excpetcourseValue').textContent = `${exceptCourse.toFixed(1)}°`;
  document.getElementById('rudderAngleValue').textContent = `${rudderAngle.toFixed(1)}°`;

    if ('BatteryVoltage' in stats) {
    const voltage = stats.BatteryVoltage;
    const bar = document.getElementById("batteryBar");
    const text = document.getElementById("batteryVoltageText");

    text.textContent = voltage.toFixed(2);

    const percentage = Math.max(0, Math.min(100, ((voltage - 22) / 3) * 100));
    bar.style.width = percentage + "%";

    if (voltage <= 23.0) {
      bar.style.backgroundColor = "red";
    } else {
      bar.style.backgroundColor = "rgb(0, 200, 0)";
    }
  }
}




function addState(stateText) {
  const history = document.getElementById('stateHistory');
  const entry = document.createElement('div');
  entry.textContent = `[${new Date().toLocaleTimeString()}] ${stateText}`;
  history.insertBefore(entry, history.firstChild);
}


