#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# Usage: sudo bash init.sh
if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo: sudo GHCR_TOKEN=<pat> bash init-pi.sh" >&2
  exit 1
fi
GHCR_TOKEN="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"
USERNAME="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

# ── System update ────────────────────────────────────────────────────────────
echo "==> Updating system packages"
apt-get update -qq
apt-get upgrade -y -qq
apt-get autoremove -y -qq

# ── Podman ───────────────────────────────────────────────────────────────────
echo "==> Installing Podman"
apt-get install -y podman
systemctl enable --now podman.socket
usermod -aG dialout "$USERNAME"
usermod -aG video "$USERNAME"

# ── BlueOS setup ────────────────────────────────────────────────────────────
echo "==> Pulling BlueOS image"
podman pull docker.io/bluerobotics/blueos-core:1.4.3

echo "==> Setting up BlueOS"
mkdir -p /usr/blueos/userdata
chown -R 1000:1000 /usr/blueos/userdata

cat > /etc/systemd/system/blueos.service << 'BLUEOS_EOF'
[Unit]
Description=BlueOS - ArduPilot Companion
After=network.target podman.socket

[Service]
Restart=on-failure
ExecStartPre=-/usr/bin/podman rm -f blueos
ExecStart=/usr/bin/podman run --rm \
  --name blueos \
  --privileged \
  --network host \
  -v /run/udev:/run/udev:ro \
  -v /run/podman/podman.sock:/var/run/docker.sock \
  -v /usr/blueos/userdata:/usr/blueos/userdata \
  -e BLUEOS_UID=1000 \
  docker.io/bluerobotics/blueos-core:1.4.3
ExecStop=/usr/bin/podman stop blueos

[Install]
WantedBy=multi-user.target
BLUEOS_EOF

systemctl daemon-reload
systemctl enable blueos.service

# ── Udev rules for USB devices ──────────────────────────────────────────────
echo "==> Setting up udev rules for LIDAR and other USB devices"
cat > /etc/udev/rules.d/99-njord.rules << 'UDEV_EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"
SUBSYSTEM=="video4linux", GROUP="video", MODE="0666"
UDEV_EOF

# ── cgroups / boot config ────────────────────────────────────────────────────
CMDLINE=/boot/firmware/cmdline.txt
[[ -f $CMDLINE ]] || CMDLINE=/boot/cmdline.txt

if ! grep -q "cgroup_memory=1" "$CMDLINE"; then
  echo "==> Enabling cgroup memory in $CMDLINE"
  sed -i 's/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/' "$CMDLINE"
fi

CONFIG=/boot/firmware/config.txt
[[ -f $CONFIG ]] || CONFIG=/boot/config.txt

if ! grep -q "^gpu_mem=" "$CONFIG"; then
  echo "==> Setting gpu_mem=16 in $CONFIG"
  echo "gpu_mem=16" >> "$CONFIG"
fi

# ── Njord container ──────────────────────────────────────────────────────────
IMAGE="ghcr.io/node-engineering-club/node-ros-2026:latest"

echo "==> Pulling Njord image"
echo "$GHCR_TOKEN" | podman login ghcr.io -u x-access-token --password-stdin
podman pull "$IMAGE"

cat > /etc/systemd/system/njord-update.service << UPDATE_EOF
[Unit]
Description=Pull latest Njord image
Before=njord.service
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo "$GHCR_TOKEN" | /usr/bin/podman login ghcr.io -u x-access-token --password-stdin && /usr/bin/podman pull $IMAGE'

[Install]
WantedBy=multi-user.target
UPDATE_EOF

cat > /etc/systemd/system/njord.service << NJORD_EOF
[Unit]
Description=Njord ROS2 Stack
After=network.target blueos.service njord-update.service
Requires=blueos.service

[Service]
Restart=on-failure
SuccessExitStatus=143
ExecStartPre=-/usr/bin/podman rm -f njord
ExecStart=/usr/bin/podman run --rm \
  --name njord \
  --init \
  --privileged \
  --network host \
  --ipc host \
  --pid host \
  $IMAGE
ExecStop=/usr/bin/podman stop njord

[Install]
WantedBy=multi-user.target
NJORD_EOF

systemctl daemon-reload
systemctl enable njord-update.service njord.service

# ── Web UI ───────────────────────────────────────────────────────────────────
echo "==> Installing Njord Web UI"
mkdir -p /opt/njord-webui

cat > /opt/njord-webui/main.py << 'WEBUI_PY_EOF'
import http.server
import json
import socketserver
import urllib.request
from pathlib import Path

PORT = 8080
MAVLINK_GPS = "http://localhost:6040/v1/mavlink/vehicles/1/components/1/messages/GLOBAL_POSITION_INT"
WEBBRIDGE   = "http://localhost:8081"


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/api/gps":
            self._proxy(MAVLINK_GPS)
        elif self.path.startswith("/api/"):
            self._proxy(WEBBRIDGE + self.path.split("?")[0])
        elif self.path in ("/", "/index.html"):
            self._serve("index.html", "text/html")
        else:
            self.send_response(404)
            self.end_headers()

    def _proxy(self, url):
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                body  = r.read()
                ctype = r.headers.get("Content-Type", "application/octet-stream")
                code  = r.status
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        except urllib.request.HTTPError as e:
            self.send_response(e.code)
            self.end_headers()
        except Exception as e:
            body = json.dumps({"error": str(e)}).encode()
            self.send_response(502)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

    def _serve(self, filename, content_type):
        path = Path(__file__).parent / filename
        if path.exists():
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


print(f"Njord dashboard running on http://0.0.0.0:{PORT}")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()
WEBUI_PY_EOF

cat > /opt/njord-webui/index.html << 'WEBUI_HTML_EOF'
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Njord Live Dashboard</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:#060d06;color:#00ff00;font-family:'Courier New',monospace;height:100vh;display:flex;flex-direction:column;overflow:hidden}
    #header{display:flex;align-items:center;gap:10px;padding:6px 14px;border-bottom:1px solid #0d2a0d;flex-shrink:0;flex-wrap:wrap}
    #title{font-size:1em;font-weight:bold;letter-spacing:.05em;margin-right:4px}
    .chip{font-size:.65em;padding:2px 7px;border-radius:3px;border:1px solid #1a4a1a;color:#555;background:#0a0f0a;transition:all .3s}
    .chip.on{color:#00ff00;border-color:#00ff00;text-shadow:0 0 6px #00ff00}
    #odom-bar{margin-left:auto;font-size:.75em;color:#aaa;display:flex;gap:16px}
    .ofield label{color:#007700;font-size:.7em;display:block}
    .ofield span{color:#00ff00}
    #grid{flex:1;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:3px;padding:3px;min-height:0}
    .panel{border:1px solid #0d2a0d;display:flex;flex-direction:column;overflow:hidden;background:#020802}
    .panel-title{font-size:.6em;color:#005500;padding:3px 8px;flex-shrink:0;letter-spacing:.12em;border-bottom:1px solid #0a1e0a}
    .panel-body{flex:1;position:relative;overflow:hidden;display:flex;align-items:center;justify-content:center}
    #cam-canvas,#seg-img{max-width:100%;max-height:100%;object-fit:contain;display:block}
    .no-signal{color:#1a4a1a;font-size:.75em;letter-spacing:.1em}
    #lidar-canvas{width:100%;height:100%;display:block}
    #map{width:100%;height:100%}
    #gps-overlay{position:absolute;top:8px;left:8px;z-index:500;background:rgba(0,0,0,.82);border:1px solid #00ff00;padding:8px 12px;font-size:.72em;min-width:160px;pointer-events:none}
    #gps-status{font-size:.7em;font-weight:bold;padding:2px 6px;border-radius:2px;margin-bottom:6px;display:inline-block;background:#440000;color:#ff4444;border:1px solid #ff4444}
    #gps-status.on{background:#004400;color:#00ff00;border-color:#00ff00}
    .gfield label{color:#007700;font-size:.7em}
    .gfield span{color:#00ff00}
    #det-count{position:absolute;top:6px;right:8px;font-size:.65em;color:#ffaa00;background:rgba(0,0,0,.75);padding:2px 6px;border:1px solid #ffaa00;border-radius:2px;z-index:10}
  </style>
</head>
<body>
<div id="header">
  <span id="title">&#9875; NJORD LIVE DASHBOARD</span>
  <span class="chip" id="chip-cam">CAM</span>
  <span class="chip" id="chip-seg">SEG</span>
  <span class="chip" id="chip-lidar">LIDAR</span>
  <span class="chip" id="chip-gps">GPS</span>
  <span class="chip" id="chip-odom">ODOM</span>
  <div id="odom-bar">
    <div class="ofield"><label>HEADING</label><span id="hdg">---</span>&deg;</div>
    <div class="ofield"><label>SPEED</label><span id="spd">---</span> m/s</div>
    <div class="ofield"><label>X</label><span id="ox">---</span> m</div>
    <div class="ofield"><label>Y</label><span id="oy">---</span> m</div>
  </div>
</div>
<div id="grid">
  <div class="panel">
    <div class="panel-title">&#9654; CAMERA FEED + YOLO DETECTIONS</div>
    <div class="panel-body">
      <canvas id="cam-canvas"></canvas>
      <div id="det-count" style="display:none">0 objects</div>
      <div class="no-signal" id="cam-nosig">NO SIGNAL</div>
    </div>
  </div>
  <div class="panel">
    <div class="panel-title">&#9654; SEGMENTATION MASK</div>
    <div class="panel-body">
      <img id="seg-img" src="" alt="" style="display:none"/>
      <div class="no-signal" id="seg-nosig">NO SIGNAL</div>
    </div>
  </div>
  <div class="panel">
    <div class="panel-title">&#9654; LIDAR SCAN (top-down)</div>
    <div class="panel-body" style="background:#020802">
      <canvas id="lidar-canvas"></canvas>
    </div>
  </div>
  <div class="panel">
    <div class="panel-title">&#9654; GPS MAP</div>
    <div class="panel-body">
      <div id="map"></div>
      <div id="gps-overlay">
        <div id="gps-status">WAITING FOR FIX...</div>
        <div class="gfield"><label>LATITUDE</label><span id="lat">--</span></div>
        <div class="gfield"><label>LONGITUDE</label><span id="lon">--</span></div>
        <div class="gfield"><label>ALTITUDE (m)</label><span id="alt">--</span></div>
        <div class="gfield"><label>REL ALT (m)</label><span id="rel_alt">--</span></div>
      </div>
    </div>
  </div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
function chip(id,state){const el=document.getElementById('chip-'+id);if(el)el.className='chip'+(state?' on':'')}
const map=L.map('map',{zoomControl:false}).setView([0,0],2);
L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png').addTo(map);
const gpsMarker=L.circleMarker([0,0],{color:'#00ff00',radius:7,fillOpacity:.85}).addTo(map);
const gpsTrack=L.polyline([],{color:'#00ff00',weight:2,opacity:.5}).addTo(map);
let firstFix=true;
async function updateGPS(){try{const res=await fetch('/api/gps');const data=await res.json();if(data.error)throw new Error(data.error);const msg=data.message;const lat=msg.lat/1e7,lon=msg.lon/1e7,alt=msg.alt/1000,rel_alt=msg.relative_alt/1000;const pos=[lat,lon];gpsMarker.setLatLng(pos);gpsTrack.addLatLng(pos);if(firstFix){map.setView(pos,16);firstFix=false}document.getElementById('gps-status').textContent='GPS ACTIVE';document.getElementById('gps-status').className='on';document.getElementById('lat').textContent=lat.toFixed(6)+'°';document.getElementById('lon').textContent=lon.toFixed(6)+'°';document.getElementById('alt').textContent=alt.toFixed(1)+' m';document.getElementById('rel_alt').textContent=rel_alt.toFixed(1)+' m';chip('gps',true)}catch{document.getElementById('gps-status').textContent='WAITING FOR FIX...';document.getElementById('gps-status').className='';chip('gps',false)}setTimeout(updateGPS,1000)}
const camCanvas=document.getElementById('cam-canvas');const camCtx=camCanvas.getContext('2d');const camNosig=document.getElementById('cam-nosig');const detCountEl=document.getElementById('det-count');let latestDets=[];
async function updateCamera(){try{const res=await fetch('/api/camera');if(res.status===204)throw new Error('no frame');if(!res.ok)throw new Error(''+res.status);const blob=await res.blob();const bmp=await createImageBitmap(blob);camCanvas.width=bmp.width;camCanvas.height=bmp.height;camCtx.drawImage(bmp,0,0);if(latestDets.length){camCtx.lineWidth=2;camCtx.font='12px Courier New';for(const d of latestDets){camCtx.strokeStyle='#00ff00';camCtx.strokeRect(d.cx-d.w/2,d.cy-d.h/2,d.w,d.h);if(d.class_id!==undefined){const lbl=d.class_id+(d.score!==undefined?' '+(d.score*100).toFixed(0)+'%':'');camCtx.fillStyle='rgba(0,0,0,.6)';camCtx.fillRect(d.cx-d.w/2,d.cy-d.h/2-16,camCtx.measureText(lbl).width+6,16);camCtx.fillStyle='#00ff00';camCtx.fillText(lbl,d.cx-d.w/2+3,d.cy-d.h/2-3)}}}camNosig.style.display='none';camCanvas.style.display='block';const n=latestDets.length;detCountEl.style.display=n?'block':'none';detCountEl.textContent=n+' object'+(n===1?'':'s');chip('cam',true)}catch{camCanvas.style.display='none';camNosig.style.display='block';chip('cam',false)}setTimeout(updateCamera,100)}
const segImg=document.getElementById('seg-img');const segNosig=document.getElementById('seg-nosig');let segUrl=null;
async function updateSeg(){try{const res=await fetch('/api/seg');if(res.status===204)throw new Error('no frame');if(!res.ok)throw new Error(''+res.status);const blob=await res.blob();const url=URL.createObjectURL(blob);if(segUrl)URL.revokeObjectURL(segUrl);segUrl=url;segImg.src=url;segImg.style.display='block';segNosig.style.display='none';chip('seg',true)}catch{segImg.style.display='none';segNosig.style.display='block';chip('seg',false)}setTimeout(updateSeg,100)}
const lidarCanvas=document.getElementById('lidar-canvas');const lidarCtx=lidarCanvas.getContext('2d');
function drawLidar(data){const w=lidarCanvas.parentElement.clientWidth||400;const h=lidarCanvas.parentElement.clientHeight||400;lidarCanvas.width=w;lidarCanvas.height=h;const cx=w/2,cy=h/2;const maxR=data.range_max||10;const scale=Math.min(w,h)*0.45/maxR;lidarCtx.fillStyle='#020802';lidarCtx.fillRect(0,0,w,h);lidarCtx.strokeStyle='#0a2a0a';lidarCtx.lineWidth=1;lidarCtx.font='10px Courier New';lidarCtx.fillStyle='#1a5a1a';for(let i=1;i<=4;i++){const r=maxR*i/4;lidarCtx.beginPath();lidarCtx.arc(cx,cy,r*scale,0,Math.PI*2);lidarCtx.stroke();lidarCtx.fillText((r%1===0?r:r.toFixed(1))+'m',cx+3,cy-r*scale+11)}lidarCtx.strokeStyle='#0d3a0d';lidarCtx.beginPath();lidarCtx.moveTo(cx,4);lidarCtx.lineTo(cx,h-4);lidarCtx.moveTo(4,cy);lidarCtx.lineTo(w-4,cy);lidarCtx.stroke();lidarCtx.strokeStyle='#005500';lidarCtx.lineWidth=2;lidarCtx.beginPath();lidarCtx.moveTo(cx,cy-18);lidarCtx.lineTo(cx-4,cy-12);lidarCtx.moveTo(cx,cy-18);lidarCtx.lineTo(cx+4,cy-12);lidarCtx.stroke();lidarCtx.fillStyle='#00aa00';lidarCtx.beginPath();lidarCtx.arc(cx,cy,4,0,Math.PI*2);lidarCtx.fill();lidarCtx.fillStyle='#00ff00';for(const pt of data.points){if(!pt)continue;const sx=cx-pt[1]*scale;const sy=cy-pt[0]*scale;if(sx<0||sx>w||sy<0||sy>h)continue;lidarCtx.fillRect(sx-1,sy-1,2,2)}}
async function updateLidar(){try{const res=await fetch('/api/lidar');if(res.ok){const data=await res.json();if(data.points){drawLidar(data);chip('lidar',true)}}else chip('lidar',false)}catch{chip('lidar',false)}setTimeout(updateLidar,120)}
async function updateOdom(){try{const res=await fetch('/api/odom');if(res.ok){const d=await res.json();document.getElementById('hdg').textContent=d.yaw_deg.toFixed(1);document.getElementById('spd').textContent=d.speed.toFixed(2);document.getElementById('ox').textContent=d.x.toFixed(1);document.getElementById('oy').textContent=d.y.toFixed(1);chip('odom',true)}}catch{chip('odom',false)}setTimeout(updateOdom,200)}
async function updateDetections(){try{const res=await fetch('/api/detections');if(res.ok){const d=await res.json();latestDets=d.detections||[]}}catch{}setTimeout(updateDetections,100)}
updateGPS();updateCamera();updateSeg();updateLidar();updateOdom();updateDetections();
</script>
</body>
</html>
WEBUI_HTML_EOF

cat > /etc/systemd/system/njord-webui.service << 'WEBUI_SVC_EOF'
[Unit]
Description=Njord GPS Web UI
After=network.target blueos.service
Wants=blueos.service

[Service]
Type=simple
Restart=on-failure
RestartSec=5
ExecStart=/usr/bin/python3 /opt/njord-webui/main.py

[Install]
WantedBy=multi-user.target
WEBUI_SVC_EOF

systemctl daemon-reload
systemctl enable njord-webui.service

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "Initialization complete. Rebooting..."
reboot
