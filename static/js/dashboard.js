// Socket.IO connection
const socket = io();

// Charts
let trafficChart, speedChart, congestionChart;
const maxDataPoints = 50;
const trafficData = { time: [], forward: [], backward: [] };
const speedData = { time: [], speed: [] };
const congestionData = { time: [], level: [] };

// Initialize charts
function initCharts() {
    const chartOptions = {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            y: { beginAtZero: true, grid: { color: '#0f3460' } },
            x: { grid: { color: '#0f3460' } }
        },
        plugins: { legend: { labels: { color: '#eee' } } }
    };

    trafficChart = new Chart(document.getElementById('trafficChart'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Forward', data: [], borderColor: '#4ecca3', tension: 0.4 },
                { label: 'Backward', data: [], borderColor: '#e74c3c', tension: 0.4 }
            ]
        },
        options: chartOptions
    });

    speedChart = new Chart(document.getElementById('speedChart'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [{ label: 'Average Speed (km/h)', data: [], borderColor: '#4ecca3', tension: 0.4 }]
        },
        options: chartOptions
    });

    congestionChart = new Chart(document.getElementById('congestionChart'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [{ label: 'Congestion Level (%)', data: [], borderColor: '#e74c3c', tension: 0.4 }]
        },
        options: chartOptions
    });
}

// Socket event handlers
socket.on('connect', () => {
    console.log('Connected to server');
});

socket.on('simulation_update', (data) => {
    updateUI(data);

    // Update median slider position from simulation (handles automatic shifts)
    if (data.median_position !== undefined) {
        const medianSlider = document.querySelector('#median input[type="range"]');
        if (medianSlider && Math.abs(parseFloat(medianSlider.value) - data.median_position) > 0.1) {
            medianSlider.value = data.median_position;
            document.getElementById('medianPos').textContent = data.median_position.toFixed(1) + 'm';
        }
    }
});

// Listen for median position updates from other clients
socket.on('median_update', (data) => {
    document.getElementById('medianPos').textContent = data.position;
    const medianSlider = document.querySelector('#median input[type="range"]');
    if (medianSlider) {
        medianSlider.value = data.position;
    }
    document.getElementById('modeDisplay').textContent = data.mode;
    showNotification(`Median shifted to ${data.position}m - Mode: ${data.mode}`);
});

// Listen for speed multiplier updates
socket.on('speed_update', (data) => {
    document.getElementById('speedValue').textContent = data.multiplier.toFixed(1);
    const speedSlider = document.querySelector('#speed input[type="range"]');
    if (speedSlider) {
        speedSlider.value = data.multiplier;
    }
    showNotification(`Speed set to ${data.multiplier.toFixed(1)}x`);
});

// Listen for camera view changes
socket.on('camera_update', (data) => {
    // Update active camera button
    document.querySelectorAll('.camera-btn').forEach(btn => {
        btn.classList.remove('active');
        if (btn.getAttribute('onclick').includes(data.view)) {
            btn.classList.add('active');
        }
    });
    showNotification(`Camera switched to ${data.view}`);
});

// Listen for weather changes
socket.on('weather_update', (data) => {
    const weatherSelect = document.getElementById('weatherSelect');
    if (weatherSelect) {
        weatherSelect.value = data.weather;
    }
    showNotification(`Weather changed to ${data.weather}`);
});

// Listen for vehicle spawn events
socket.on('vehicle_spawned', (data) => {
    showNotification(`${data.count} vehicles spawned (${data.direction})`);
});

// Listen for simulation start/stop events
socket.on('simulation_started', (data) => {
    document.getElementById('startBtn').disabled = true;
    document.getElementById('stopBtn').disabled = false;
    showNotification('Simulation started successfully');
});

socket.on('simulation_stopped', (data) => {
    document.getElementById('startBtn').disabled = false;
    document.getElementById('stopBtn').disabled = true;
    showNotification('Simulation stopped');
});

// Update UI with simulation data
function updateUI(data) {
    document.getElementById('modeDisplay').textContent = data.mode;
    document.getElementById('totalVehicles').textContent = data.total_vehicles;
    document.getElementById('forwardSpeed').textContent = data.forward_speed.toFixed(1);
    document.getElementById('congestionLevel').textContent = data.congestion_level.toFixed(1);
    document.getElementById('timeElapsed').textContent = data.time_elapsed.toFixed(0);

    // Update status
    const statusIndicator = document.getElementById('statusIndicator');
    const statusText = document.getElementById('statusText');
    if (data.running) {
        statusIndicator.className = 'status-indicator status-running';
        statusText.textContent = 'Running';
    } else {
        statusIndicator.className = 'status-indicator status-stopped';
        statusText.textContent = 'Stopped';
    }

    // Update lane counts
    if (data.lane_data) {
        document.getElementById('lane_f1').textContent = data.lane_data.forward[0] || 0;
        document.getElementById('lane_f2').textContent = data.lane_data.forward[1] || 0;
        document.getElementById('lane_f3').textContent = data.lane_data.forward[2] || 0;
        document.getElementById('lane_f4').textContent = data.lane_data.forward[3] || 0;
        document.getElementById('lane_b1').textContent = data.lane_data.backward[0] || 0;
        document.getElementById('lane_b2').textContent = data.lane_data.backward[1] || 0;
        document.getElementById('lane_b3').textContent = data.lane_data.backward[2] || 0;
        document.getElementById('lane_b4').textContent = data.lane_data.backward[3] || 0;
    }

    // Update charts
    updateCharts(data);
}

function updateCharts(data) {
    const time = data.time_elapsed.toFixed(0);

    // Traffic chart
    trafficData.time.push(time);
    trafficData.forward.push(data.forward_vehicles);
    trafficData.backward.push(data.backward_vehicles);

    if (trafficData.time.length > maxDataPoints) {
        trafficData.time.shift();
        trafficData.forward.shift();
        trafficData.backward.shift();
    }

    trafficChart.data.labels = trafficData.time;
    trafficChart.data.datasets[0].data = trafficData.forward;
    trafficChart.data.datasets[1].data = trafficData.backward;
    trafficChart.update('none');

    // Speed chart
    speedData.time.push(time);
    speedData.speed.push(data.forward_speed);

    if (speedData.time.length > maxDataPoints) {
        speedData.time.shift();
        speedData.speed.shift();
    }

    speedChart.data.labels = speedData.time;
    speedChart.data.datasets[0].data = speedData.speed;
    speedChart.update('none');

    // Congestion chart
    congestionData.time.push(time);
    congestionData.level.push(data.congestion_level);

    if (congestionData.time.length > maxDataPoints) {
        congestionData.time.shift();
        congestionData.level.shift();
    }

    congestionChart.data.labels = congestionData.time;
    congestionChart.data.datasets[0].data = congestionData.level;
    congestionChart.update('none');
}

// Panel navigation
function showPanel(panelId) {
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.menu-item').forEach(m => m.classList.remove('active'));

    document.getElementById(panelId).classList.add('active');
    event.target.closest('.menu-item').classList.add('active');
}

// API functions
async function startSimulation() {
    try {
        document.getElementById('startBtn').disabled = true;
        const response = await fetch('/api/start', { method: 'POST' });
        const data = await response.json();

        if (response.ok) {
            document.getElementById('stopBtn').disabled = false;
            console.log('Simulation started:', data.message);
        } else {
            document.getElementById('startBtn').disabled = false;
            showNotification('Error: ' + (data.error || 'Failed to start'));
            console.error('Failed to start simulation:', data);
        }
    } catch (error) {
        document.getElementById('startBtn').disabled = false;
        showNotification('Error: Could not connect to server');
        console.error('Start simulation error:', error);
    }
}

async function stopSimulation() {
    try {
        document.getElementById('stopBtn').disabled = true;
        const response = await fetch('/api/stop', { method: 'POST' });
        const data = await response.json();

        if (response.ok) {
            document.getElementById('startBtn').disabled = false;
            console.log('Simulation stopped:', data.message);
        } else {
            document.getElementById('stopBtn').disabled = false;
            showNotification('Error: ' + (data.error || 'Failed to stop'));
            console.error('Failed to stop simulation:', data);
        }
    } catch (error) {
        document.getElementById('stopBtn').disabled = false;
        showNotification('Error: Could not connect to server');
        console.error('Stop simulation error:', error);
    }
}

async function shiftMedian(direction, amount) {
    // Update local slider immediately for responsive feedback
    const slider = document.querySelector('input[type="range"][oninput*="updateMedianPos"]');
    if (slider) {
        slider.value = amount;
    }
    document.getElementById('medianPos').textContent = amount;

    // Send to server (will broadcast to all clients)
    await fetch('/api/median/shift', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction, amount })
    });
}

function updateMedianPos(value) {
    document.getElementById('medianPos').textContent = value;
    // Send update to server in real-time as user drags slider
    fetch('/api/median/shift', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: 'manual', amount: parseFloat(value) })
    }).catch(err => console.error('Failed to update median:', err));
}

async function spawnVehicles(direction) {
    const count = direction === 'forward' ?
        document.getElementById('forwardCount').value :
        document.getElementById('backwardCount').value;

    await fetch(`/api/spawn/${direction}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ count: parseInt(count) })
    });
}

async function setSpeedMultiplier(multiplier) {
    document.getElementById('speedValue').textContent = multiplier.toFixed(1);
    await fetch('/api/speed/set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ multiplier })
    });
}

function updateSpeed(value) {
    setSpeedMultiplier(parseFloat(value));
}

async function switchCamera(view) {
    document.querySelectorAll('.camera-btn').forEach(b => b.classList.remove('active'));
    event.target.closest('.camera-btn').classList.add('active');

    await fetch('/api/camera/switch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ view })
    });
}

async function setWeather(weather) {
    await fetch('/api/weather/set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ weather })
    });
}

async function setTime(time) {
    // Implementation needed
}

async function setTrafficLights(state) {
    await fetch('/api/traffic-lights/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ state })
    });
}

async function createCongestion(direction) {
    const intensity = document.getElementById('congestionIntensity').value;
    await fetch('/api/congestion/create', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction, intensity: parseFloat(intensity) })
    });
}

async function clearCongestion() {
    // Implementation needed
}

async function clearAllVehicles() {
    // Implementation needed
}

function toggleAutoMode() {
    const auto = document.getElementById('autoMode').checked;
    fetch('/api/mode/toggle', { method: 'POST' });
}

// Notification system
function showNotification(message) {
    const notification = document.createElement('div');
    notification.className = 'notification';
    notification.innerHTML = `<i class="fas fa-info-circle"></i> ${message}`;
    document.body.appendChild(notification);

    setTimeout(() => {
        notification.style.animation = 'slideOut 0.3s ease-out';
        setTimeout(() => notification.remove(), 300);
    }, 3000);
}

// Initialize on load
window.onload = () => {
    initCharts();
};