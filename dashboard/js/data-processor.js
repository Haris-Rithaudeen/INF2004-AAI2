/**
 * Data Processing and State Management
 */

// Global state
const dashboardState = {
    telemetry: {
        leftSpeed: 0,
        rightSpeed: 0,
        leftDistance: 0,
        rightDistance: 0
    },
    compass: {
        heading: 0,
        mx: 0,
        my: 0,
        mz: 0
    },
    sensors: {
        ultrasonic: 0,
        irLine: 0,
        onLine: false
    },
    barcode: {
        value: '',
        barCount: 0
    },
    state: {
        current: 'IDLE',
        command: '',
        duration: 0
    },
    pid: {
        error: 0,
        kp: 0,
        ki: 0,
        kd: 0,
        output: 0
    },
    obstacle: {
        distance: 0,
        servoAngle: 90,
        chosenPath: 'NONE',
        status: 'CLEAR'
    }
};

// Data history for charts
const dataHistory = {
    speed: {
        labels: [],
        left: [],
        right: []
    },
    pidError: {
        labels: [],
        data: []
    },
    imuComparison: {
        labels: [],
        raw: [],
        filtered: []
    },
    movementTimeline: {
        states: []
    }
};

// Process telemetry data
function processTelemetry(data) {
    dashboardState.telemetry = {
        leftSpeed: data.ls || 0,
        rightSpeed: data.rs || 0,
        leftDistance: data.ld || 0,
        rightDistance: data.rd || 0
    };
    
    // Update UI
    updateElement('leftSpeed', `${data.ls.toFixed(1)} cm/s`);
    updateElement('rightSpeed', `${data.rs.toFixed(1)} cm/s`);
    updateElement('leftDistance', `${data.ld.toFixed(0)} cm`);
    updateElement('rightDistance', `${data.rd.toFixed(0)} cm`);
    
    // Update gauges
    if (window.gauges) {
        updateGauge(window.gauges.leftMotor, data.ls, 100);
        updateGauge(window.gauges.rightMotor, data.rs, 100);
    }
    
    // Update speed chart
    const now = new Date().toLocaleTimeString();
    addChartData(dataHistory.speed, now, data.ls, data.rs);
    if (window.charts && window.charts.speedChart) {
        updateSpeedChart(window.charts.speedChart, dataHistory.speed);
    }
    
    // Update compass from telemetry if available
    if (data.hd !== undefined) {
        updateCompass(data.hd);
    }
    
    // Check success criteria
    checkDemo1Criteria();
}

// Process sensor data
function processSensors(data) {
    dashboardState.sensors = {
        ultrasonic: data.ultrasonic || 0,
        irLine: data.ir_line || 0,
        onLine: data.on_line || false
    };
    
    // Update UI
    updateElement('ultrasonicValue', `${data.ultrasonic.toFixed(2)} cm`);
    updateElement('irLineValue', data.ir_line);
    
    const lineStatus = document.getElementById('lineStatus');
    if (lineStatus) {
        lineStatus.textContent = data.on_line ? 'ON LINE' : 'OFF LINE';
        lineStatus.classList.toggle('online', data.on_line);
    }
    
    // Update ultrasonic progress bar
    const ultrasonicProgress = document.getElementById('ultrasonicProgress');
    if (ultrasonicProgress) {
        const percentage = Math.min(100, (data.ultrasonic / 400) * 100);
        ultrasonicProgress.style.width = `${percentage}%`;
    }
    
    // Check for obstacle alert
    if (data.ultrasonic < 20 && data.ultrasonic > 0) {
        addEventLog('warning', `Obstacle detected at ${data.ultrasonic.toFixed(1)}cm`);
    }
    
    // Check success criteria
    checkDemo2Criteria();
}

// Process compass data
function processCompass(data) {
    dashboardState.compass = {
        heading: data.heading || 0,
        mx: data.mx || 0,
        my: data.my || 0,
        mz: data.mz || 0
    };
    
    updateCompass(data.heading);
    updateElement('magX', data.mx);
    updateElement('magY', data.my);
    updateElement('magZ', data.mz);
}

// Process barcode data
function processBarcode(data) {
    dashboardState.barcode = {
        value: data.barcode || '',
        barCount: data.bars || 0
    };
    
    updateElement('barcodeValue', data.barcode || 'No data');
    updateElement('barCount', data.bars || 0);
    
    if (data.barcode && data.barcode !== 'No data') {
        addEventLog('info', `Barcode scanned: ${data.barcode}`);
        addCommandHistory(data.barcode);
    }
}

// Process state machine data
function processState(data) {
    const previousState = dashboardState.state.current;
    
    dashboardState.state = {
        current: data.state || 'IDLE',
        command: data.command || '',
        duration: 0
    };
    
    updateElement('currentState', data.state || 'IDLE');
    updateElement('sysState', data.state || 'IDLE');
    updateElement('lastCommand', data.command || 'NONE');
    
    // Update state transitions visualization
    document.querySelectorAll('.transition-item').forEach(item => {
        item.classList.toggle('active', item.dataset.state === data.state);
    });
    
    // Log state change
    if (previousState !== data.state) {
        addEventLog('info', `State changed: ${previousState} → ${data.state}`);
        
        // Add to movement timeline
        dataHistory.movementTimeline.states.push({
            state: data.state,
            time: Date.now()
        });
    }
    
    // Check success criteria
    checkDemo2Criteria();
}

// Process PID controller data
function processPID(data) {
    dashboardState.pid = {
        error: data.error || 0,
        kp: data.kp || 0,
        ki: data.ki || 0,
        kd: data.kd || 0,
        output: data.output || 0
    };
    
    updateElement('pidKp', data.kp.toFixed(3));
    updateElement('pidKi', data.ki.toFixed(3));
    updateElement('pidKd', data.kd.toFixed(3));
    updateElement('pidError', data.error.toFixed(3));
    updateElement('pidOutput', data.output.toFixed(3));
    
    // Update PID error chart
    const now = new Date().toLocaleTimeString();
    addChartData(dataHistory.pidError, now, data.error);
    if (window.charts && window.charts.pidErrorChart) {
        updatePIDChart(window.charts.pidErrorChart, dataHistory.pidError);
    }
    
    // Check success criteria
    checkDemo1Criteria();
}

// Process obstacle detection data
function processObstacle(data) {
    dashboardState.obstacle = {
        distance: data.distance || 0,
        servoAngle: data.servo_angle || 90,
        chosenPath: data.chosen_path || 'NONE',
        status: data.status || 'CLEAR'
    };
    
    updateElement('obstacleDistance', `${data.distance.toFixed(2)} cm`);
    updateElement('servoAngle', `${data.servo_angle}°`);
    updateElement('chosenPath', data.chosen_path || 'NONE');
    
    // Update obstacle indicator
    const indicator = document.getElementById('obstacleIndicator');
    if (indicator) {
        indicator.className = 'status-indicator-large';
        const indicatorText = indicator.querySelector('.indicator-text');
        
        if (data.distance < 20) {
            indicator.classList.add('danger');
            indicatorText.textContent = 'OBSTACLE';
        } else if (data.distance < 50) {
            indicator.classList.add('warning');
            indicatorText.textContent = 'CAUTION';
        } else {
            indicatorText.textContent = 'CLEAR';
        }
    }
    
    // Update servo beam visualization
    updateServoBeam(data.servo_angle);
    
    // Check success criteria
    checkDemo3Criteria();
}

// Helper function to update DOM element
function updateElement(id, value) {
    const element = document.getElementById(id);
    if (element) {
        element.textContent = value;
    }
}

// Helper function to add data to chart history
function addChartData(history, label, ...values) {
    if (history.labels.length >= CONFIG.charts.maxDataPoints) {
        history.labels.shift();
        Object.keys(history).forEach(key => {
            if (key !== 'labels' && Array.isArray(history[key])) {
                history[key].shift();
            }
        });
    }
    
    history.labels.push(label);
    
    if (history.data !== undefined) {
        history.data.push(values[0]);
    } else {
        history.left.push(values[0]);
        history.right.push(values[1]);
    }
}

// Add event to log
function addEventLog(type, message) {
    const eventLog = document.getElementById('eventLog');
    if (!eventLog) return;
    
    const eventItem = document.createElement('div');
    eventItem.className = `event-item event-${type}`;
    
    const now = new Date();
    const timeString = now.toLocaleTimeString();
    
    eventItem.innerHTML = `
        <span class="event-time">${timeString}</span>
        <span class="event-message">${message}</span>
    `;
    
    eventLog.insertBefore(eventItem, eventLog.firstChild);
    
    // Limit log size
    while (eventLog.children.length > CONFIG.ui.eventLogMaxItems) {
        eventLog.removeChild(eventLog.lastChild);
    }
}

// Clear event log
function clearEventLog() {
    const eventLog = document.getElementById('eventLog');
    if (eventLog) {
        eventLog.innerHTML = '';
        addEventLog('info', 'Event log cleared');
    }
}

// Add command to history
function addCommandHistory(command) {
    const historyList = document.getElementById('commandHistory');
    if (!historyList) return;
    
    const historyItem = document.createElement('div');
    historyItem.className = 'history-item';
    historyItem.innerHTML = `
        <span>${command}</span>
        <span style="color: var(--color-text-tertiary); font-size: var(--font-size-xs);">
            ${new Date().toLocaleTimeString()}
        </span>
    `;
    
    if (historyList.querySelector('.history-empty')) {
        historyList.innerHTML = '';
    }
    
    historyList.insertBefore(historyItem, historyList.firstChild);
    
    // Limit history size
    while (historyList.children.length > CONFIG.ui.commandHistoryMax) {
        historyList.removeChild(historyList.lastChild);
    }
}


// Update compass visualization
function updateCompass(heading) {
    updateElement('headingValue', heading.toFixed(1));
    
    // Update compass needle
    const needle = document.getElementById('compassNeedle');
    if (needle) {
        needle.style.transform = `rotate(${heading}deg)`;
        needle.style.transformOrigin = '100px 100px';
    }
    
    // Update direction text
    let direction = 'North';
    if (heading >= 337.5 || heading < 22.5) direction = 'North';
    else if (heading >= 22.5 && heading < 67.5) direction = 'Northeast';
    else if (heading >= 67.5 && heading < 112.5) direction = 'East';
    else if (heading >= 112.5 && heading < 157.5) direction = 'Southeast';
    else if (heading >= 157.5 && heading < 202.5) direction = 'South';
    else if (heading >= 202.5 && heading < 247.5) direction = 'Southwest';
    else if (heading >= 247.5 && heading < 292.5) direction = 'West';
    else direction = 'Northwest';
    
    updateElement('headingDirection', direction);
}

// Update servo beam visualization
function updateServoBeam(angle) {
    const beam = document.getElementById('servoBeam');
    if (beam) {
        beam.style.transform = `rotate(${angle - 90}deg)`;
    }
}