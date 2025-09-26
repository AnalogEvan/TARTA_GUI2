let padTarget = null;
let currentInput = null;
let viewMode = 'single';
let scanTotals = {cycles:0, sparks:0}, cleanTotals = {cycles:0, sparks:0}, pmTotals = {sparks:0};
let pmTimerInterval, pmTimeSinceSpark;
let detectedUsbPath = null;
let lastScreenId = 'welcome-screen';
let hourlyCountdownInterval = null; // CHANGED: Added variable for the countdown timer

const screenTitles = {
  'welcome-screen': 'Welcome',
  'menu-screen': 'Expert Menu',
  'community-screen': 'Community Menu',
  'scan-screen': 'Expert Scan',
  'clean-screen': 'Expert Clean',
  'pm-screen': 'Expert PM Monitoring',
  'view-screen': 'View Spectra',
  'hourly-monitoring-screen': 'Hourly Monitoring',
  'community-standard-screen': 'Community Regular Scan',
  'community-adaptive-screen': 'Community Adaptive PM'
};

let communityConfig = null;

window.onload = () => {
  setupNumPad();
  eel.is_rpi_ready()((ok) => {
    updateStatus('menu-status', ok);
    updateStatus('community-status', ok);
  });
  eel.get_config()((cfg) => {
    communityConfig = cfg;
    setupCommunityScreens();
  });
};

function showScreen(id) {
  const currentScreen = document.querySelector('.screen.active');
  if (currentScreen) {
      lastScreenId = currentScreen.id;
  }
  document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  document.getElementById('header-title').textContent = screenTitles[id] || '';
}

function goBack() {
    showScreen(lastScreenId);
}

function exitProgram() {
    eel.close_app()();
    window.close();
}

function updateStatus(elId, connected) {
  const el = document.getElementById(elId);
  el.textContent = connected ? 'Ready' : 'Error';
  el.classList.toggle('connected', connected);
  el.classList.toggle('disconnected', !connected);
}

eel.expose(update_ui, 'update_ui');
function update_ui(msg) {
  console.log("Message from Python:", msg);

  if (msg === 'DONE' || msg.includes('STOP')) {
    setScreenLock(false);
    document.getElementById('scan-progress').style.display = 'none';
    document.getElementById('clean-progress').style.display = 'none';
    clearInterval(pmTimerInterval);
    clearInterval(hourlyCountdownInterval); // CHANGED: Stop countdown on abort/finish
    id('hourly-monitor-next-event').textContent = '-'; // CHANGED: Reset countdown text

    const statusText = msg.includes('STOP') ? 'Operation stopped.' : 'Operation finished.';
    ['scan-status', 'clean-status', 'pm-status', 'cscan-status', 'cpm-status', 'hourly-monitor-status'].forEach(id => setStatus(id, statusText));
    return;
  }
  
  const [type, ...rest] = msg.split(',');
  const val = rest.join(','); // Handle potential commas in the value
  
  if (type === 'CYCLE') {
    id('scan-cycle').textContent = `Cycle: ${val}/${scanTotals.cycles}`;
    id('clean-cycle').textContent = `Cycle: ${val}/${cleanTotals.cycles}`;
  } else if (type === 'SPARK') {
    id('scan-spark').textContent = `Spark: ${val}/${scanTotals.sparks}`;
    id('clean-spark').textContent = `Spark: ${val}/${cleanTotals.sparks}`;
  } else if (type === 'TIME_LEFT') {
    let m = Math.floor((+val)/60000);
    let s = Math.floor(((+val)%60000)/1000);
    id('scan-time').textContent = `Time left: ${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
  } else if (type === 'HOURLY_MONITOR_STATUS') {
    const [status, nextEvent] = val.split(',');
    setStatus('hourly-monitor-status', status);
    // The next event text is now handled by the live countdown
  } else if (type === 'HOURLY_NEXT_EVENT') {
    // CHANGED: New logic to handle countdown timer updates
    startHourlyCountdown(val);
  } else {
    // This block handles PM monitoring
    const activeScreen = document.querySelector('.screen.active').id;
    const prefix = activeScreen.includes('community') ? 'cpm' : 'pm';

    if (type === 'PM_VALUE') {
        const cur = +val;
        id(prefix + '-current').textContent = cur;
        id(prefix + '-bar').value = cur;
        setStatus(prefix + '-status', `Monitoring... Current Value: ${cur.toFixed(0)}`);
    } else if (msg.startsWith('SPARK,')) {
        pmTimeSinceSpark = 0;
        const sparkNum = msg.split(',')[1];
        setStatus(prefix + '-status', `Sparking: ${sparkNum}/${pmTotals.sparks}`);
    } else if (msg === 'PM THRESHOLD REACHED') {
        setStatus(prefix + '-status', 'Threshold reached, starting sparks...');
    } else if (msg === 'PM SPARKS COMPLETE') {
        setStatus(prefix + '-status', 'Sparks complete, restarting monitoring.');
    }
  }
}

// CHANGED: New function to manage the countdown timer
function startHourlyCountdown(isoString) {
  clearInterval(hourlyCountdownInterval); // Clear any old timer

  const targetTime = new Date(isoString);
  const nextEventEl = id('hourly-monitor-next-event');

  hourlyCountdownInterval = setInterval(() => {
    const now = new Date();
    const totalSeconds = Math.round((targetTime - now) / 1000);

    if (totalSeconds <= 0) {
      clearInterval(hourlyCountdownInterval);
      nextEventEl.textContent = "Event in progress...";
      return;
    }

    const hours = Math.floor(totalSeconds / 3600);
    const minutes = Math.floor((totalSeconds % 3600) / 60);
    const seconds = totalSeconds % 60;

    // Display as HH:MM:SS
    nextEventEl.textContent =
      `${String(hours).padStart(2, '0')}:` +
      `${String(minutes).padStart(2, '0')}:` +
      `${String(seconds).padStart(2, '0')}`;
  }, 1000);
}

function startHourlyMonitoring() {
  eel.start_hourly_monitoring()().then(ok => {
    if (ok) {
      setScreenLock(true, 'hourly-monitoring-screen');
      setStatus('hourly-monitor-status', 'Hourly Monitoring Started');
    } else {
      setStatus('hourly-monitor-status', 'Failed to start. Another operation may be running.');
    }
  });
}

function startScan() {
  const d = +id('scan-duration').value;
  const s = +id('scan-sparks').value;
  const c = +id('scan-cycles').value;
  if (!d || !s || !c) return setStatus('scan-status', 'Invalid parameters');
  
  eel.start_scan(d, s, c)().then(ok => {
    if (ok) {
      setScreenLock(true, 'scan-screen');
      setStatus('scan-status', 'Scan started');
      scanTotals = {cycles:c, sparks:s};
      id('scan-cycle').textContent = `Cycle: 0/${c}`;
      id('scan-spark').textContent = `Spark: 0/${s}`;
      id('scan-time').textContent = `Time left: ${String(d).padStart(2,'0')}:00`;
      document.getElementById('scan-progress').style.display = 'flex';
    } else {
      setStatus('scan-status', 'Failed to start. Another operation may be running.');
    }
  });
}

function startClean() {
  const s = +id('clean-sparks').value;
  if (!s) return setStatus('clean-status','Invalid sparks');

  eel.start_clean(s)().then(ok => {
    if (ok) {
      setScreenLock(true, 'clean-screen');
      setStatus('clean-status', 'Clean started');
      cleanTotals = {cycles:1, sparks:s};
      id('clean-cycle').textContent = `Cycle: 0/1`;
      id('clean-spark').textContent = `Spark: 0/${s}`;
      document.getElementById('clean-progress').style.display = 'flex';
    } else {
      setStatus('clean-status','Failed to start. Another operation may be running.');
    }
  });
}

function startPM(isCommunity = false) {
  const prefix = isCommunity ? 'cpm' : 'pm';
  const t = +id(prefix + '-threshold').value;
  const s = +id(prefix + '-sparks').value;
  const type = id(prefix + '-type').value;

  if (!t || !s) return setStatus(prefix + '-status', 'Invalid parameters');
  
  pmTotals.sparks = s;
  id(prefix + '-bar').max = t;
  id(prefix + '-max').textContent = '/' + t;
  id(prefix + '-current').textContent = '0';
  id(prefix + '-bar').value = 0;

  eel.start_pm(s, t, type)().then(ok => {
    if (ok) {
      setScreenLock(true, isCommunity ? 'community-adaptive-screen' : 'pm-screen');
      setStatus(prefix + '-status', 'PM Monitoring Started');
      startPmTimer(prefix);
    } else {
      setStatus(prefix + '-status', 'Failed to start. Another operation may be running.');
    }
  });
}

function startCommunityScan() {
  const mult = +id('cscan-duration').value;
  const d = mult * communityConfig.duration;
  const s = communityConfig.sparks;
  const c = communityConfig.cycles;
  if (!mult || mult <= 0) return setStatus('cscan-status', 'Invalid duration multiplier');

  eel.start_scan(d, s, c)().then(ok => {
     if (ok) {
      setScreenLock(true, 'community-standard-screen');
      setStatus('cscan-status', 'Community Scan started');
      scanTotals = {cycles:c, sparks:s};
    } else {
      setStatus('cscan-status', 'Failed to start. Another operation may be running.');
    }
  });
}

function startPmTimer(prefix) {
  pmTimeSinceSpark = 0;
  clearInterval(pmTimerInterval);
  
  id(prefix + '-timer').textContent = `Time Since Last Spark: ${pmTimeSinceSpark}s`;
  pmTimerInterval = setInterval(() => {
    pmTimeSinceSpark++;
    id(prefix + '-timer').textContent = `Time Since Last Spark: ${pmTimeSinceSpark}s`;
  }, 1000);
}

function abortOp() {
  clearInterval(pmTimerInterval);
  clearInterval(hourlyCountdownInterval); // CHANGED: Stop countdown on abort
  eel.abort_all()();
}

function refreshList() {
  eel.list_scans()((files) => {
    const sel = id('scan-list');
    sel.innerHTML = '';
    files.forEach(f => {
      const filename = f.split(/[/\\]/).pop();
      sel.add(new Option(filename, f));
    });
    if (files.length && viewMode === 'single') {
      loadScan(files[0]);
    }
  });
}

function loadScan(path) {
  eel.get_scan_data(path)((data) => {
    if (!data || !data.x || !data.y || data.x.length === 0) {
      console.error("Invalid or empty data:", data);
      id('scan-status').textContent = "Failed to load data";
      return;
    }

    const trace = {x: data.x, y: data.y, mode: 'lines', name: 'Spectrum'};
    const peaks = {
      x: data.peaks.map(i => data.x[i]),
      y: data.peaks.map(i => data.y[i]),
      mode: 'markers',
      marker: {color: 'red', size: 6},
      name: 'Peaks'
    };

    Plotly.newPlot('plot', [trace, peaks], {
      margin: {t: 30},
      xaxis: {title: 'Wavelength'},
      yaxis: {title: 'Intensity'}
    });
  });
}

function setupNumPad() {
  const keys = ['1','2','3','4','5','6','7','8','9','0','DEL','OK'];
  const container = document.querySelector('.keys');
  keys.forEach(k => {
    const btn = document.createElement('button'); btn.textContent = k;
    btn.onclick = () => handleKey(k);
    container.appendChild(btn);
  });
}

function openNumPad(input) {
  currentInput = input;
  document.getElementById('pad-context').textContent = input.previousSibling.textContent || '';
  document.getElementById('pad-display').value = input.value;
  document.getElementById('num-pad').classList.remove('hidden');
}

function handleKey(key) {
  let val = id('pad-display').value;
  if (key === 'DEL') val = val.slice(0, -1);
  else if (key === 'OK') return closeNumPad();
  else val += key;
  id('pad-display').value = val;
}

function closeNumPad() {
  currentInput.value = id('pad-display').value;
  id('num-pad').classList.add('hidden');
}

function id(i) { return document.getElementById(i); }
function setStatus(el, txt) { id(el).textContent = txt; }

document.addEventListener('DOMContentLoaded', () => {
  document.querySelectorAll('.btn').forEach(button => {
    button.addEventListener('click', function (e) {
      const existing = button.querySelector('.ripple');
      if (existing) existing.remove();
      const ripple = document.createElement('span');
      ripple.classList.add('ripple');
      const rect = button.getBoundingClientRect();
      const size = Math.max(rect.width, rect.height);
      ripple.style.width = ripple.style.height = `${size * 2}px`;
      ripple.style.left = `${e.clientX - rect.left - size}px`;
      ripple.style.top = `${e.clientY - rect.top - size}px`;
      button.appendChild(ripple);
      setTimeout(() => ripple.remove(), 600);
    });
  });
});

function setupCommunityScreens() {
  id('cscan-sparks').value = communityConfig.sparks;
  id('cscan-cycles').value = communityConfig.cycles;
  id('cscan-sparks').disabled = true;
  id('cscan-cycles').disabled = true;

  const durSelect = id('cscan-duration');
  durSelect.innerHTML = '';
  [1, 2, 3, 4, 5, 10].forEach(mult => durSelect.add(new Option(`${mult}×`, mult)));
  
  const updateTotalTime = () => {
    const total = +durSelect.value * communityConfig.duration;
    id('cscan-duration-info').textContent = `Total scan time = ${total} minutes`;
  };
  durSelect.onchange = updateTotalTime;
  updateTotalTime();

  id('cpm-type').value = communityConfig.pm_type;
  id('cpm-threshold').value = communityConfig.threshold;
  id('cpm-sparks').value = communityConfig.sparks;
  ['cpm-type', 'cpm-threshold', 'cpm-sparks'].forEach(idName => id(idName).disabled = true);
}

function setScreenLock(locked, screenId = null) {
    const screens = screenId ? [document.getElementById(screenId)] : document.querySelectorAll('.screen');
    screens.forEach(screen => {
        screen.querySelectorAll('input, select, button').forEach(el => {
            const isAbortOrBack = el.textContent.includes('Abort') || el.textContent.includes('Back') || el.textContent.includes('View');
            el.disabled = locked && !isAbortOrBack;
        });
    });
}


function loadAverage() {
  eel.get_scan_data_avg()((data) => {
    if (!data.x.length) {
      id('scan-status').textContent = "No scans to average.";
      return;
    }
    const trace = {x: data.x, y: data.y, mode: 'lines', name: 'Average'};
    const peaks = {
      x: data.peaks.map(i => data.x[i]),
      y: data.peaks.map(i => data.y[i]),
      mode: 'markers',
      name: 'Peaks',
      marker: {color: 'red', size: 6}
    };
    Plotly.newPlot('plot', [trace, peaks], {
      margin: {t: 30},
      xaxis: {title: 'Wavelength'},
      yaxis: {title: 'Intensity'}
    });
  });
}

// --- USB Functions ---

eel.expose(show_usb_prompt, 'show_usb_prompt');
function show_usb_prompt(path) {
  detectedUsbPath = path;
  const driveName = path.split(/[/\\]/).pop(); // Get last part of the path
  id('usb-drive-info').textContent = `Save all output data to drive "${driveName}"?`;
  id('usb-confirm-modal').classList.remove('hidden');
}

function confirmUsbSave(shouldSave) {
  id('usb-confirm-modal').classList.add('hidden');
  if (shouldSave && detectedUsbPath) {
    showToast("Copying files to USB...");
    eel.copy_data_to_usb(detectedUsbPath)();
  }
  detectedUsbPath = null; // Clear path after decision
}

eel.expose(usb_copy_status, 'usb_copy_status');
function usb_copy_status(status, message) {
    showToast(message, status === 'error' ? 5000 : 3000);
}

function showToast(message, duration = 3000) {
  const toast = id('notification-toast');
  toast.textContent = message;
  toast.classList.add('show');
  setTimeout(() => {
    toast.classList.remove('show');
  }, duration);
}
