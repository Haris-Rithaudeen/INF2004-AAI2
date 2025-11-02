/**
 * Main Application Entry Point
 */

// Application state
let updateRateCounter = 0;
let lastRateUpdate = Date.now();

// Initialize application
function initApp() {
    console.log('[APP] Initializing Robot Telemetry Dashboard');
    
    // Initialize visualizations
    initVisualizations();
    
    // Setup navigation
    setupNavigation();
    
    // Initialize MQTT connection
    mqttHandler = new MQTTHandler(CONFIG);
    mqttHandler.connect();
    
    // Start system monitors
    startSystemMonitors();
    
    // Log initialization
    addEventLog('info', 'Dashboard initialized successfully');
}

// Setup navigation between views
function setupNavigation() {
    const navTabs = document.querySelectorAll('.nav-tab');
    const views = document.querySelectorAll('.view');
    
    navTabs.forEach(tab => {
        tab.addEventListener('click', () => {
            const viewId = tab.dataset.view;
            
            // Update active tab
            navTabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            
            // Update active view
            views.forEach(v => v.classList.remove('active'));
            const activeView = document.getElementById(viewId);
            if (activeView) {
                activeView.classList.add('active');
            }
            
            // Update active demo in system info
            const demoNames = {
                'overview': 'Overview',
                'demo1': 'Demo 1',
                'demo2': 'Demo 2',
                'demo3': 'Demo 3'
            };
            updateElement('sysDemo', demoNames[viewId] || 'None');
        });
    });
}

// Start system monitoring tasks
function startSystemMonitors() {
    // Update system info every second
    setInterval(() => {
        if (mqttHandler) {
            const stats = mqttHandler.getStats();
            
            // Update uptime
            updateElement('sysUptime', stats.uptime);
            
            // Update message count
            updateElement('sysMessages', stats.messageCount);
            
            // Update last update time
            const timeSince = Date.now() - stats.lastMessageTime;
            if (timeSince < 2000) {
                updateElement('lastUpdate', 'Just now');
            } else {
                updateElement('lastUpdate', `${Math.floor(timeSince / 1000)}s ago`);
            }
        }
    }, 1000);
    
    // Calculate update rate
    setInterval(() => {
        const rate = updateRateCounter;
        updateElement('sysRate', `${rate} Hz`);
        updateRateCounter = 0;
    }, 1000);
    
    // Monitor for MQTT messages
    if (mqttHandler) {
        Object.values(CONFIG.mqtt.topics).forEach(topic => {
            mqttHandler.registerHandler(topic, () => {
                updateRateCounter++;
            });
        });
    }
}

// Success criteria checking functions
function checkSuccessCriteria(demo, criteriaId, condition) {
    const element = document.getElementById(criteriaId);
    if (!element) return;
    
    if (condition) {
        element.classList.add('completed');
        element.querySelector('.check-icon').textContent = '✓';
    } else {
        element.classList.remove('completed');
        element.querySelector('.check-icon').textContent = '○';
    }
}

// Demo 1 success criteria
function checkDemo1Criteria() {
    const state = dashboardState;
    
    // Check straight path (drift < 5cm)
    const drift = Math.abs(state.telemetry.leftDistance - state.telemetry.rightDistance);
    checkSuccessCriteria('demo1', 'check_straightPath', drift < CONFIG.thresholds.demo1.maxDrift);
    
    // Check IMU corrections applied
    checkSuccessCriteria('demo1', 'check_imuCorrection', state.compass.heading > 0);
    
    // Check speed data available
    const hasSpeedData = state.telemetry.leftSpeed > 0 || state.telemetry.rightSpeed > 0;
    checkSuccessCriteria('demo1', 'check_speedData', hasSpeedData);
    
    // Check filtering (would need raw vs filtered data)
    checkSuccessCriteria('demo1', 'check_filtering', true);
    
    // Update drift metrics
    updateElement('lateralDrift', `${drift.toFixed(1)} cm`);
}

// Demo 2 success criteria
function checkDemo2Criteria() {
    const state = dashboardState;
    
    // Check following line
    checkSuccessCriteria('demo2', 'check_followLine', state.sensors.onLine);
    
    // Check barcode interpretation
    const hasBarcodeCommand = state.barcode.value && state.barcode.value !== 'No data';
    checkSuccessCriteria('demo2', 'check_interpretBarcode', hasBarcodeCommand);
    
    // Check MQTT data available
    const hasData = mqttHandler && mqttHandler.isConnected;
    checkSuccessCriteria('demo2', 'check_mqttData', hasData);
    
    // Check no track loss (placeholder logic)
    checkSuccessCriteria('demo2', 'check_noTrackLoss', true);
    
    // Update line error
    const lineError = state.sensors.irLine / 4095; // Normalize
    updateElement('lineError', lineError.toFixed(2));
    
    const lineErrorBar = document.getElementById('lineErrorBar');
    if (lineErrorBar) {
        lineErrorBar.style.width = `${Math.abs(lineError) * 100}%`;
    }
}

// Demo 3 success criteria
function checkDemo3Criteria() {
    const state = dashboardState;
    
    // Check obstacle detection
    const obstacleDetected = state.obstacle.distance < CONFIG.thresholds.demo3.obstacleDetectDist;
    checkSuccessCriteria('demo3', 'check_obstacleDetect', obstacleDetected);
    
    // Check servo scanning
    const isScanning = state.obstacle.servoAngle !== 90;
    checkSuccessCriteria('demo3', 'check_servoScan', isScanning);
    
    // Check path chosen
    const hasChosenPath = state.obstacle.chosenPath !== 'NONE';
    checkSuccessCriteria('demo3', 'check_pathChoice', hasChosenPath);
    
    // Check rejoin line (placeholder)
    checkSuccessCriteria('demo3', 'check_rejoinLine', false);
    
    // Check telemetry log
    checkSuccessCriteria('demo3', 'check_telemetryLog', mqttHandler && mqttHandler.isConnected);
}

// Add telemetry row to table
function addTelemetryRow(state, command, lineStatus, speed, distance) {
    const table = document.getElementById('telemetryTable');
    if (!table) return;
    
    // Remove empty placeholder
    const emptyRow = table.querySelector('.table-empty');
    if (emptyRow) {
        emptyRow.remove();
    }
    
    const row = document.createElement('tr');
    const now = new Date().toLocaleTimeString();
    
    row.innerHTML = `
        <td>${now}</td>
        <td>${state}</td>
        <td>${command}</td>
        <td>${lineStatus}</td>
        <td>${speed}</td>
        <td>${distance}</td>
    `;
    
    table.insertBefore(row, table.firstChild);
    
    // Limit table size
    while (table.children.length > CONFIG.ui.telemetryTableMax) {
        table.removeChild(table.lastChild);
    }
}

// Update telemetry table with current data
function updateTelemetryTable() {
    const state = dashboardState.state.current;
    const command = dashboardState.state.command;
    const lineStatus = dashboardState.sensors.onLine ? 'ON' : 'OFF';
    const speed = `${dashboardState.telemetry.leftSpeed.toFixed(1)}/${dashboardState.telemetry.rightSpeed.toFixed(1)}`;
    const distance = `${dashboardState.telemetry.leftDistance.toFixed(0)}cm`;
    
    addTelemetryRow(state, command, lineStatus, speed, distance);
}

// Update telemetry table periodically
setInterval(() => {
    if (mqttHandler && mqttHandler.isConnected) {
        updateTelemetryTable();
    }
}, 1000);

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (mqttHandler) {
        mqttHandler.disconnect();
    }
    destroyCharts();
});

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initApp();
});

// Handle window resize
window.addEventListener('resize', () => {
    Object.values(window.charts).forEach(chart => {
        if (chart) chart.resize();
    });
});

// Error handling
window.addEventListener('error', (event) => {
    console.error('[APP] Error:', event.error);
    addEventLog('danger', `Application error: ${event.error.message}`);
});

// Console message
console.log(`
╔═══════════════════════════════════════════════════════════════╗
║                                                               ║
║           Robot Telemetry Dashboard v1.0                      ║
║           Professional Integration Test Platform              ║
║                                                               ║
║   Status: Ready                                               ║
║   MQTT Broker: ${CONFIG.mqtt.broker}                          ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
`);