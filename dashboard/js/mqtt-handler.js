/**
 * MQTT Connection Handler
 */

class MQTTHandler {
    constructor(config) {
        this.config = config;
        this.client = null;
        this.isConnected = false;
        this.connectionStartTime = null;
        this.messageCount = 0;
        this.lastMessageTime = Date.now();
        this.messageHandlers = new Map();
    }

    connect() {
        console.log(`[MQTT] Connecting to ${this.config.mqtt.broker}`);
        
        this.client = mqtt.connect(this.config.mqtt.broker, this.config.mqtt.options);
        
        this.client.on('connect', () => this.onConnect());
        this.client.on('disconnect', () => this.onDisconnect());
        this.client.on('error', (err) => this.onError(err));
        this.client.on('reconnect', () => this.onReconnect());
        this.client.on('message', (topic, message) => this.onMessage(topic, message));
    }

    onConnect() {
        console.log('[MQTT] Connected successfully');
        this.isConnected = true;
        this.connectionStartTime = Date.now();
        
        // Update UI
        this.updateConnectionStatus(true);
        
        // Subscribe to all topics
        Object.values(this.config.mqtt.topics).forEach(topic => {
            this.client.subscribe(topic, (err) => {
                if (!err) {
                    console.log(`[MQTT] Subscribed to ${topic}`);
                } else {
                    console.error(`[MQTT] Subscribe failed for ${topic}:`, err);
                }
            });
        });

        // Add event log entry
        addEventLog('info', 'MQTT connected to broker');
    }

    onDisconnect() {
        console.log('[MQTT] Disconnected');
        this.isConnected = false;
        this.updateConnectionStatus(false);
        addEventLog('warning', 'MQTT disconnected from broker');
    }

    onError(err) {
        console.error('[MQTT] Error:', err);
        addEventLog('danger', `MQTT error: ${err.message}`);
    }

    onReconnect() {
        console.log('[MQTT] Reconnecting...');
        addEventLog('info', 'MQTT attempting to reconnect');
    }

    onMessage(topic, message) {
        this.messageCount++;
        this.lastMessageTime = Date.now();
        
        try {
            const data = JSON.parse(message.toString());
            
            // Call registered handlers
            if (this.messageHandlers.has(topic)) {
                this.messageHandlers.get(topic).forEach(handler => {
                    handler(data);
                });
            }
            
            // Route to appropriate processor
            this.routeMessage(topic, data);
            
        } catch (err) {
            console.error('[MQTT] Message parse error:', err);
        }
    }

    routeMessage(topic, data) {
        const topics = this.config.mqtt.topics;
        
        if (topic === topics.telemetry) {
            processTelemetry(data);
        } else if (topic === topics.sensors) {
            processSensors(data);
        } else if (topic === topics.compass) {
            processCompass(data);
        } else if (topic === topics.barcode) {
            processBarcode(data);
        } else if (topic === topics.state) {
            processState(data);
        } else if (topic === topics.pid) {
            processPID(data);
        } else if (topic === topics.obstacle) {
            processObstacle(data);
        }
    }

    registerHandler(topic, handler) {
        if (!this.messageHandlers.has(topic)) {
            this.messageHandlers.set(topic, []);
        }
        this.messageHandlers.get(topic).push(handler);
    }

    updateConnectionStatus(connected) {
        const statusDot = document.getElementById('connectionStatus');
        const statusLabel = document.getElementById('connectionLabel');
        const sysConnection = document.getElementById('sysConnection');
        
        if (connected) {
            statusDot.classList.add('connected');
            statusLabel.textContent = 'Connected';
            if (sysConnection) sysConnection.textContent = 'Online';
        } else {
            statusDot.classList.remove('connected');
            statusLabel.textContent = 'Disconnected';
            if (sysConnection) sysConnection.textContent = 'Offline';
        }
    }

    getUptime() {
        if (!this.connectionStartTime) return '0s';
        
        const seconds = Math.floor((Date.now() - this.connectionStartTime) / 1000);
        const minutes = Math.floor(seconds / 60);
        const hours = Math.floor(minutes / 60);
        
        if (hours > 0) {
            return `${hours}h ${minutes % 60}m`;
        } else if (minutes > 0) {
            return `${minutes}m ${seconds % 60}s`;
        } else {
            return `${seconds}s`;
        }
    }

    getStats() {
        return {
            connected: this.isConnected,
            uptime: this.getUptime(),
            messageCount: this.messageCount,
            lastMessageTime: this.lastMessageTime
        };
    }

    disconnect() {
        if (this.client) {
            this.client.end();
        }
    }
}

// Global MQTT handler instance
let mqttHandler = null;