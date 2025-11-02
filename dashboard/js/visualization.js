/**
 * Chart and Visualization Management
 */

// Global chart instances
window.charts = {};
window.gauges = {};

// Initialize all visualizations
function initVisualizations() {
    initGauges();
    initCharts();
}

// Initialize gauge charts
function initGauges() {
    const gaugeConfig = {
        type: 'doughnut',
        options: {
            responsive: true,
            maintainAspectRatio: true,
            cutout: '75%',
            rotation: -90,
            circumference: 180,
            plugins: {
                legend: { display: false },
                tooltip: { enabled: false }
            },
            animation: {
                animateRotate: true,
                animateScale: false
            }
        }
    };

    // Left motor gauge
    const leftCanvas = document.getElementById('leftMotorGauge');
    if (leftCanvas) {
        window.gauges.leftMotor = new Chart(leftCanvas, {
            ...gaugeConfig,
            data: {
                datasets: [{
                    data: [0, 100],
                    backgroundColor: [
                        CONFIG.charts.colors.primary,
                        'rgba(255, 255, 255, 0.1)'
                    ],
                    borderWidth: 0
                }]
            }
        });
    }

    // Right motor gauge
    const rightCanvas = document.getElementById('rightMotorGauge');
    if (rightCanvas) {
        window.gauges.rightMotor = new Chart(rightCanvas, {
            ...gaugeConfig,
            data: {
                datasets: [{
                    data: [0, 100],
                    backgroundColor: [
                        CONFIG.charts.colors.secondary,
                        'rgba(255, 255, 255, 0.1)'
                    ],
                    borderWidth: 0
                }]
            }
        });
    }
}

// Initialize line charts
function initCharts() {
    const defaultOptions = {
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
            mode: 'index',
            intersect: false
        },
        plugins: {
            legend: {
                display: true,
                position: 'top',
                labels: {
                    color: CONFIG.charts.colors.text,
                    usePointStyle: true,
                    padding: 15
                }
            }
        },
        scales: {
            y: {
                beginAtZero: true,
                grid: {
                    color: CONFIG.charts.colors.grid
                },
                ticks: {
                    color: CONFIG.charts.colors.text
                }
            },
            x: {
                grid: {
                    color: CONFIG.charts.colors.grid
                },
                ticks: {
                    color: CONFIG.charts.colors.text,
                    maxRotation: 0,
                    autoSkip: true,
                    maxTicksLimit: 10
                }
            }
        }
    };

    // Speed history chart
    const speedCanvas = document.getElementById('speedChart');
    if (speedCanvas) {
        window.charts.speedChart = new Chart(speedCanvas, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Left Speed',
                        data: [],
                        borderColor: CONFIG.charts.colors.primary,
                        backgroundColor: hexToRgba(CONFIG.charts.colors.primary, 0.1),
                        tension: 0.4,
                        fill: true,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Right Speed',
                        data: [],
                        borderColor: CONFIG.charts.colors.secondary,
                        backgroundColor: hexToRgba(CONFIG.charts.colors.secondary, 0.1),
                        tension: 0.4,
                        fill: true,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                ...defaultOptions,
                scales: {
                    ...defaultOptions.scales,
                    y: {
                        ...defaultOptions.scales.y,
                        max: 100,
                        title: {
                            display: true,
                            text: 'Speed (cm/s)',
                            color: CONFIG.charts.colors.text
                        }
                    }
                }
            }
        });
    }

    // PID error chart
    const pidCanvas = document.getElementById('pidErrorChart');
    if (pidCanvas) {
        window.charts.pidErrorChart = new Chart(pidCanvas, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'PID Error',
                    data: [],
                    borderColor: CONFIG.charts.colors.warning,
                    backgroundColor: hexToRgba(CONFIG.charts.colors.warning, 0.1),
                    tension: 0.4,
                    fill: true,
                    pointRadius: 0,
                    borderWidth: 2
                }]
            },
            options: {
                ...defaultOptions,
                scales: {
                    ...defaultOptions.scales,
                    y: {
                        ...defaultOptions.scales.y,
                        title: {
                            display: true,
                            text: 'Error',
                            color: CONFIG.charts.colors.text
                        }
                    }
                }
            }
        });
    }

    // IMU comparison chart
    const imuCanvas = document.getElementById('imuComparisonChart');
    if (imuCanvas) {
        window.charts.imuComparisonChart = new Chart(imuCanvas, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Raw IMU',
                        data: [],
                        borderColor: CONFIG.charts.colors.danger,
                        backgroundColor: hexToRgba(CONFIG.charts.colors.danger, 0.1),
                        tension: 0.1,
                        fill: false,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    {
                        label: 'Filtered IMU',
                        data: [],
                        borderColor: CONFIG.charts.colors.primary,
                        backgroundColor: hexToRgba(CONFIG.charts.colors.primary, 0.1),
                        tension: 0.4,
                        fill: true,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                ...defaultOptions,
                scales: {
                    ...defaultOptions.scales,
                    y: {
                        ...defaultOptions.scales.y,
                        title: {
                            display: true,
                            text: 'Heading (degrees)',
                            color: CONFIG.charts.colors.text
                        }
                    }
                }
            }
        });
    }

    // Movement timeline chart
    const timelineCanvas = document.getElementById('movementTimeline');
    if (timelineCanvas) {
        window.charts.movementTimeline = new Chart(timelineCanvas, {
            type: 'bar',
            data: {
                labels: [],
                datasets: [{
                    label: 'State Duration',
                    data: [],
                    backgroundColor: CONFIG.charts.colors.primary,
                    borderWidth: 0
                }]
            },
            options: {
                ...defaultOptions,
                indexAxis: 'y',
                scales: {
                    x: {
                        ...defaultOptions.scales.y,
                        title: {
                            display: true,
                            text: 'Duration (ms)',
                            color: CONFIG.charts.colors.text
                        }
                    },
                    y: {
                        ...defaultOptions.scales.x,
                        grid: {
                            display: false
                        }
                    }
                }
            }
        });
    }
}

// Update gauge value
function updateGauge(gauge, value, max = 100) {
    if (!gauge) return;
    
    const percentage = Math.min(100, (value / max) * 100);
    gauge.data.datasets[0].data = [percentage, 100 - percentage];
    gauge.update('none');
}

// Update speed chart
function updateSpeedChart(chart, history) {
    if (!chart) return;
    
    chart.data.labels = history.labels;
    chart.data.datasets[0].data = history.left;
    chart.data.datasets[1].data = history.right;
    chart.update('none');
}

// Update PID chart
function updatePIDChart(chart, history) {
    if (!chart) return;
    
    chart.data.labels = history.labels;
    chart.data.datasets[0].data = history.data;
    chart.update('none');
}

// Update IMU comparison chart
function updateIMUChart(chart, history) {
    if (!chart) return;
    
    chart.data.labels = history.labels;
    chart.data.datasets[0].data = history.raw;
    chart.data.datasets[1].data = history.filtered;
    chart.update('none');
}

// Update movement timeline chart
function updateMovementTimelineChart(chart, states) {
    if (!chart || states.length === 0) return;
    
    const stateGroups = {};
    states.forEach(state => {
        if (!stateGroups[state.state]) {
            stateGroups[state.state] = 0;
        }
        stateGroups[state.state]++;
    });
    
    chart.data.labels = Object.keys(stateGroups);
    chart.data.datasets[0].data = Object.values(stateGroups);
    chart.update('none');
}

// Helper function to convert hex to rgba
function hexToRgba(hex, alpha) {
    const r = parseInt(hex.slice(1, 3), 16);
    const g = parseInt(hex.slice(3, 5), 16);
    const b = parseInt(hex.slice(5, 7), 16);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

// Destroy all charts (for cleanup)
function destroyCharts() {
    Object.values(window.charts).forEach(chart => {
        if (chart) chart.destroy();
    });
    Object.values(window.gauges).forEach(gauge => {
        if (gauge) gauge.destroy();
    });
    window.charts = {};
    window.gauges = {};
}