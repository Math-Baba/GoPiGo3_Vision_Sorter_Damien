import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import json
import urllib.parse
import time
import base64

# Config 4 couleurs
colors = {
    'green':  {'low': [48, 70, 60],  'high': [85, 255, 255],  'box': (0, 255, 0)},
    'blue':   {'low': [90, 70, 60],  'high': [130, 255, 255], 'box': (255, 100, 0)},
    'yellow': {'low': [20, 80, 80],  'high': [48, 255, 255],  'box': (0, 255, 255)},
    'red':    {'low': [0, 80, 80],   'high': [8, 255, 255],   'box': (0, 0, 255)},
    'red2':   {'low': [170, 80, 80], 'high': [179, 255, 255], 'box': (0, 0, 255)},
}
min_area = 800
mode = 'all'
latest_frame = None
lock = threading.Lock()

kernel_open = np.ones((5, 5), np.uint8)
kernel_close = np.ones((15, 15), np.uint8)


def process_color(enhanced, frame, color_key):
    cfg = colors[color_key]
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(cfg['low']), np.array(cfg['high']))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
    mask = cv2.dilate(mask, kernel_open, iterations=1)
    return mask


def find_contours(mask, frame, color_name, box_color):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            label = color_name.upper()
            cv2.putText(frame, f'{label} {int(area)}px', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 2)
            frame_cx = frame.shape[1] // 2
            d = "CENTRE"
            if cx < frame_cx - 40:
                d = "< GAUCHE"
            elif cx > frame_cx + 40:
                d = "DROITE >"
            cv2.putText(frame, d, (cx - 30, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
            detections.append({'color': color_name, 'x': cx, 'y': cy, 'area': int(area)})
    return detections


def camera_loop():
    global latest_frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        cv2.line(frame, (frame.shape[1] // 2, 0),
                 (frame.shape[1] // 2, frame.shape[0]), (100, 100, 100), 1)

        mask_combined = np.zeros(frame.shape[:2], dtype=np.uint8)
        all_detections = []

        active = []
        if mode in ['green', 'all']:
            active.append(('green', 'green', (0, 255, 0)))
        if mode in ['blue', 'all']:
            active.append(('blue', 'blue', (255, 100, 0)))
        if mode in ['yellow', 'all']:
            active.append(('yellow', 'yellow', (0, 255, 255)))
        if mode in ['red', 'all']:
            active.append(('red', 'red', (0, 0, 255)))

        for color_key, color_name, box_color in active:
            m = process_color(enhanced, frame, color_key)
            if color_key == 'red':
                m2 = process_color(enhanced, frame, 'red2')
                m = cv2.bitwise_or(m, m2)
            dets = find_contours(m, frame, color_name, box_color)
            mask_combined = cv2.bitwise_or(mask_combined, m)
            all_detections += dets

        mask_colored = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([frame, mask_colored])

        _, jpg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 80])
        b64 = base64.b64encode(jpg.tobytes()).decode('utf-8')
        with lock:
            latest_frame = json.dumps({'img': b64, 'detections': all_detections})
        time.sleep(0.05)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        global mode, min_area
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(PAGE.encode())
        elif self.path == '/frame':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with lock:
                self.wfile.write((latest_frame or '{}').encode())
        elif self.path.startswith('/set?'):
            params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            if 'color' in params:
                c = params['color'][0]
                if c in colors:
                    colors[c]['low'] = [int(params['hl'][0]), int(params['sl'][0]), int(params['vl'][0])]
                    colors[c]['high'] = [int(params['hh'][0]), int(params['sh'][0]), int(params['vh'][0])]
            if 'area' in params:
                min_area = int(params['area'][0])
            if 'mode' in params:
                mode = params['mode'][0]
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'ok': True}).encode())
        elif self.path == '/values':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            export = {}
            for k, v in colors.items():
                export[k] = {'low': v['low'], 'high': v['high']}
            self.wfile.write(json.dumps(export, indent=2).encode())

    def log_message(self, format, *args):
        pass


PAGE = """<!DOCTYPE html>
<html><head><title>HSV Calibration Pro v3</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',Arial,sans-serif;background:#111;color:#eee;padding:10px}
h1{text-align:center;color:#4CAF50;margin-bottom:8px;font-size:20px}
#view{text-align:center;margin:8px 0}
#view img{border:2px solid #333;border-radius:8px;max-width:100%}
.controls{display:flex;gap:10px;max-width:1100px;margin:8px auto;flex-wrap:wrap;justify-content:center}
.panel{background:#1e1e1e;border:1px solid #333;border-radius:8px;padding:10px;min-width:220px;flex:1}
.panel h3{margin-bottom:6px;font-size:13px}
.panel.green h3{color:#4CAF50}.panel.blue h3{color:#42A5F5}
.panel.yellow h3{color:#FFD600}.panel.red h3{color:#EF5350}
.sr{display:flex;align-items:center;margin:3px 0}
.sr label{width:45px;font-size:11px;color:#aaa}
.sr input[type=range]{flex:1;margin:0 4px}.sr span{width:30px;font-size:11px;text-align:right;font-family:monospace}
.btns{text-align:center;margin:8px 0}
.btn{border:none;padding:6px 14px;margin:2px;border-radius:5px;cursor:pointer;font-size:12px;font-weight:bold;color:white}
.btn.g{background:#4CAF50}.btn.g.active{background:#2E7D32;box-shadow:0 0 8px #4CAF50}
.btn.b{background:#2196F3}.btn.b.active{background:#1565C0;box-shadow:0 0 8px #2196F3}
.btn.y{background:#F9A825}.btn.y.active{background:#F57F17;box-shadow:0 0 8px #FFD600}
.btn.r{background:#E53935}.btn.r.active{background:#B71C1C;box-shadow:0 0 8px #EF5350}
.btn.all{background:#FF9800}.btn.all.active{background:#E65100;box-shadow:0 0 8px #FF9800}
#det{background:#1e1e1e;border:1px solid #333;border-radius:8px;padding:8px;max-width:1100px;margin:6px auto;font-family:monospace;font-size:11px;min-height:25px;text-align:center}
#vals{background:#222;border-radius:5px;padding:6px;max-width:1100px;margin:4px auto;font-family:monospace;font-size:10px;text-align:center;word-break:break-all}
.export-btn{background:#555;color:white;border:none;padding:5px 12px;border-radius:4px;cursor:pointer;font-size:11px;margin:5px}
.export-btn:hover{background:#777}
</style></head><body>
<h1>HSV Calibration Pro v3 - 4 Couleurs</h1>
<div class="btns">
<button class="btn g" onclick="setMode('green')">VERT</button>
<button class="btn b" onclick="setMode('blue')">BLEU</button>
<button class="btn y" onclick="setMode('yellow')">JAUNE</button>
<button class="btn r" onclick="setMode('red')">ROUGE</button>
<button class="btn all active" onclick="setMode('all')">TOUTES</button>
</div>
<div id="view"><img id="feed"></div>
<div id="det">En attente...</div>
<div class="controls">
<div class="panel green"><h3>VERT</h3>
<div class="sr"><label>H min</label><input type="range" id="green_hl" min="0" max="179" value="48" oninput="send('green')"><span id="green_hlv">48</span></div>
<div class="sr"><label>S min</label><input type="range" id="green_sl" min="0" max="255" value="70" oninput="send('green')"><span id="green_slv">70</span></div>
<div class="sr"><label>V min</label><input type="range" id="green_vl" min="0" max="255" value="60" oninput="send('green')"><span id="green_vlv">60</span></div>
<div class="sr"><label>H max</label><input type="range" id="green_hh" min="0" max="179" value="85" oninput="send('green')"><span id="green_hhv">85</span></div>
<div class="sr"><label>S max</label><input type="range" id="green_sh" min="0" max="255" value="255" oninput="send('green')"><span id="green_shv">255</span></div>
<div class="sr"><label>V max</label><input type="range" id="green_vh" min="0" max="255" value="255" oninput="send('green')"><span id="green_vhv">255</span></div>
</div>
<div class="panel blue"><h3>BLEU</h3>
<div class="sr"><label>H min</label><input type="range" id="blue_hl" min="0" max="179" value="90" oninput="send('blue')"><span id="blue_hlv">90</span></div>
<div class="sr"><label>S min</label><input type="range" id="blue_sl" min="0" max="255" value="70" oninput="send('blue')"><span id="blue_slv">70</span></div>
<div class="sr"><label>V min</label><input type="range" id="blue_vl" min="0" max="255" value="60" oninput="send('blue')"><span id="blue_vlv">60</span></div>
<div class="sr"><label>H max</label><input type="range" id="blue_hh" min="0" max="179" value="130" oninput="send('blue')"><span id="blue_hhv">130</span></div>
<div class="sr"><label>S max</label><input type="range" id="blue_sh" min="0" max="255" value="255" oninput="send('blue')"><span id="blue_shv">255</span></div>
<div class="sr"><label>V max</label><input type="range" id="blue_vh" min="0" max="255" value="255" oninput="send('blue')"><span id="blue_vhv">255</span></div>
</div>
<div class="panel yellow"><h3>JAUNE</h3>
<div class="sr"><label>H min</label><input type="range" id="yellow_hl" min="0" max="179" value="20" oninput="send('yellow')"><span id="yellow_hlv">20</span></div>
<div class="sr"><label>S min</label><input type="range" id="yellow_sl" min="0" max="255" value="80" oninput="send('yellow')"><span id="yellow_slv">80</span></div>
<div class="sr"><label>V min</label><input type="range" id="yellow_vl" min="0" max="255" value="80" oninput="send('yellow')"><span id="yellow_vlv">80</span></div>
<div class="sr"><label>H max</label><input type="range" id="yellow_hh" min="0" max="179" value="48" oninput="send('yellow')"><span id="yellow_hhv">48</span></div>
<div class="sr"><label>S max</label><input type="range" id="yellow_sh" min="0" max="255" value="255" oninput="send('yellow')"><span id="yellow_shv">255</span></div>
<div class="sr"><label>V max</label><input type="range" id="yellow_vh" min="0" max="255" value="255" oninput="send('yellow')"><span id="yellow_vhv">255</span></div>
</div>
<div class="panel red"><h3>ROUGE (plage 1: 0-8)</h3>
<div class="sr"><label>H min</label><input type="range" id="red_hl" min="0" max="179" value="0" oninput="send('red')"><span id="red_hlv">0</span></div>
<div class="sr"><label>S min</label><input type="range" id="red_sl" min="0" max="255" value="80" oninput="send('red')"><span id="red_slv">80</span></div>
<div class="sr"><label>V min</label><input type="range" id="red_vl" min="0" max="255" value="80" oninput="send('red')"><span id="red_vlv">80</span></div>
<div class="sr"><label>H max</label><input type="range" id="red_hh" min="0" max="179" value="8" oninput="send('red')"><span id="red_hhv">8</span></div>
<div class="sr"><label>S max</label><input type="range" id="red_sh" min="0" max="255" value="255" oninput="send('red')"><span id="red_shv">255</span></div>
<div class="sr"><label>V max</label><input type="range" id="red_vh" min="0" max="255" value="255" oninput="send('red')"><span id="red_vhv">255</span></div>
<h3 style="margin-top:8px">ROUGE (plage 2: 170-179)</h3>
<div class="sr"><label>H min</label><input type="range" id="red2_hl" min="0" max="179" value="170" oninput="send('red2')"><span id="red2_hlv">170</span></div>
<div class="sr"><label>S min</label><input type="range" id="red2_sl" min="0" max="255" value="80" oninput="send('red2')"><span id="red2_slv">80</span></div>
<div class="sr"><label>V min</label><input type="range" id="red2_vl" min="0" max="255" value="80" oninput="send('red2')"><span id="red2_vlv">80</span></div>
<div class="sr"><label>H max</label><input type="range" id="red2_hh" min="0" max="179" value="179" oninput="send('red2')"><span id="red2_hhv">179</span></div>
<div class="sr"><label>S max</label><input type="range" id="red2_sh" min="0" max="255" value="255" oninput="send('red2')"><span id="red2_shv">255</span></div>
<div class="sr"><label>V max</label><input type="range" id="red2_vh" min="0" max="255" value="255" oninput="send('red2')"><span id="red2_vhv">255</span></div>
</div>
</div>
<div class="controls">
<div class="panel"><h3 style="color:#FF9800">FILTRAGE</h3>
<div class="sr"><label>Aire min</label><input type="range" id="area" min="100" max="10000" value="800" oninput="sendArea()"><span id="areav">800</span></div>
</div>
</div>
<div style="text-align:center">
<button class="export-btn" onclick="exportValues()">Exporter les valeurs</button>
<button class="export-btn" onclick="window.open('/values')">JSON brut</button>
</div>
<div id="vals"></div>
<script>
let currentMode='all';
function setMode(m){
currentMode=m;
document.querySelectorAll('.btn').forEach(b=>b.classList.remove('active'));
if(m=='all')document.querySelector('.btn.all').classList.add('active');
else if(m=='green')document.querySelector('.btn.g').classList.add('active');
else if(m=='blue')document.querySelector('.btn.b').classList.add('active');
else if(m=='yellow')document.querySelector('.btn.y').classList.add('active');
else if(m=='red')document.querySelector('.btn.r').classList.add('active');
fetch('/set?mode='+m);
}
setMode('all');

function send(color){
['hl','sl','vl','hh','sh','vh'].forEach(id=>{
  let el=document.getElementById(color+'_'+id);
  if(el)document.getElementById(color+'_'+id+'v').textContent=el.value;
});
let q='color='+color;
q+='&hl='+document.getElementById(color+'_hl').value;
q+='&sl='+document.getElementById(color+'_sl').value;
q+='&vl='+document.getElementById(color+'_vl').value;
q+='&hh='+document.getElementById(color+'_hh').value;
q+='&sh='+document.getElementById(color+'_sh').value;
q+='&vh='+document.getElementById(color+'_vh').value;
q+='&area='+document.getElementById('area').value;
fetch('/set?'+q);
updateVals();
}

function sendArea(){
document.getElementById('areav').textContent=document.getElementById('area').value;
fetch('/set?area='+document.getElementById('area').value);
}

function updateVals(){
let out=[];
['green','blue','yellow','red','red2'].forEach(c=>{
  let el=document.getElementById(c+'_hl');
  if(el){
    let vals=['hl','sl','vl','hh','sh','vh'].map(id=>document.getElementById(c+'_'+id).value);
    out.push(c.toUpperCase()+': ['+vals.slice(0,3).join(',')+'] - ['+vals.slice(3).join(',')+']');
  }
});
document.getElementById('vals').textContent=out.join(' | ');
}
updateVals();

function exportValues(){
let out='# Valeurs HSV pour cube_detector.py\\n\\n';
['green','blue','yellow'].forEach(c=>{
  let vals=['hl','sl','vl','hh','sh','vh'].map(id=>document.getElementById(c+'_'+id).value);
  out+=c+": low=["+vals.slice(0,3).join(', ')+"] high=["+vals.slice(3).join(', ')+"]\\n";
});
let rv1=['hl','sl','vl','hh','sh','vh'].map(id=>document.getElementById('red_'+id).value);
let rv2=['hl','sl','vl','hh','sh','vh'].map(id=>document.getElementById('red2_'+id).value);
out+="red1: low=["+rv1.slice(0,3).join(', ')+"] high=["+rv1.slice(3).join(', ')+"]\\n";
out+="red2: low=["+rv2.slice(0,3).join(', ')+"] high=["+rv2.slice(3).join(', ')+"]\\n";
out+="\\nAire min: "+document.getElementById('area').value;
alert(out);
}

let colorMap={'green':'#4CAF50','blue':'#42A5F5','yellow':'#FFD600','red':'#EF5350'};
function poll(){
fetch('/frame').then(r=>r.json()).then(d=>{
if(d.img)document.getElementById('feed').src='data:image/jpeg;base64,'+d.img;
if(d.detections){
let det=document.getElementById('det');
if(d.detections.length==0)det.innerHTML='<span style="color:#666">Aucun cube detecte</span>';
else det.innerHTML=d.detections.map(x=>
'<span style="color:'+(colorMap[x.color]||'#fff')+'">'+x.color.toUpperCase()+' x:'+x.x+' y:'+x.y+' ('+x.area+'px)</span>').join(' &nbsp;|&nbsp; ');
}
setTimeout(poll,80);
}).catch(()=>setTimeout(poll,500));
}
poll();
</script></body></html>"""

threading.Thread(target=camera_loop, daemon=True).start()
print("HSV Calibration Pro v3 sur http://11.255.255.201:8081")
print("4 couleurs: vert, bleu, jaune, rouge")
HTTPServer(('0.0.0.0', 8081), Handler).serve_forever()