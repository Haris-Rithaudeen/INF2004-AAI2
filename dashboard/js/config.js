/**
 * Robot Telemetry Dashboard Configuration
 */

const CONFIG = {
    // MQTT Broker Settings
    mqtt: {
        broker: 'ws://172.20.10.2:9001',
        options: {
            clientId: `robot_dashboard_${Math.random().toString(16).substr(2, 8)}`,
            clean: true,
            reconnectPeriod: 1000,
            connectTimeout: 30000
        },
        topics: {
            telemetry: 'robot/telemetry',
            sensors: 'robot/sensors',
            compass: 'robot/compass',
            barcode: 'robot/barcode',
            state: 'robot/state',
            pid: 'robot/pid',
            obstacle: 'robot/obstacle'
        }
    },

    // Chart Settings
    charts: {
        maxDataPoints: 50,
        updateInterval: 100,
        colors: {
            primary: '#0066FF',
            secondary: '#2EA043',
            warning: '#F97316',
            danger: '#EF4444',
            grid: '#30363D',
            text: '#8B92A8'
        }
    },

    // Success Criteria Thresholds
    thresholds: {
        demo1: {
            maxDrift: 5.0,              // cm
            maxAngularDev: 3.0,         // degrees
            minDataRate: 0.5            // Hz
        },
        demo2: {
            maxLineError: 0.3,          // normalized
            maxTrackLoss: 2,            // count
            minTurnAccuracy: 5.0        // degrees
        },
        demo3: {
            obstacleDetectDist: 20.0,   // cm
            minClearance: 15.0,         // cm
            scanTimeout: 5000           // ms
        }
    },

    // UI Settings
    ui: {
        eventLogMaxItems: 100,
        commandHistoryMax: 20,
        telemetryTableMax: 20,
        updateRateWindow: 1000          // ms
    }
};

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = CONFIG;
}