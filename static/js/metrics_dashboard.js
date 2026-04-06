let metricsData = null;
let historyData = [];
let charts = {};

// Load metrics on page load
document.addEventListener('DOMContentLoaded', function () {
    loadMetrics();
});

async function loadMetrics() {
    try {
        // Fetch current metrics
        const currentResponse = await fetch('/api/metrics/current');
        const currentData = await currentResponse.json();

        // Fetch history
        const historyResponse = await fetch('/api/metrics/history');
        const historyDataResponse = await historyResponse.json();

        if (currentData.success) {
            metricsData = currentData.metrics;
            historyData = historyDataResponse.history || [];

            // Store conditions globally for comparison table
            if (historyData.length > 0) {
                window.currentConditions = historyData[historyData.length - 1].conditions;
            }

            renderDashboard();
        }
    } catch (error) {
        console.error('Error loading metrics:', error);
        document.getElementById('content').innerHTML = `
                    <div class="no-data">
                        <i class="fas fa-exclamation-triangle" style="font-size: 3em; color: #ff6464; margin-bottom: 20px;"></i>
                        <p>Error loading metrics data. Please ensure the simulation has been run.</p>
                    </div>
                `;
    }
}

function renderDashboard() {
    if (!metricsData) {
        document.getElementById('content').innerHTML = '<div class="no-data">No metrics data available. Run a simulation first.</div>';
        return;
    }

    const content = document.getElementById('content');
    content.innerHTML = `
                <!-- Key Metrics Cards -->
                <div class="metrics-grid">
                    <div class="metric-card">
                        <div class="metric-icon"><i class="fas fa-clock"></i></div>
                        <div class="metric-title">System Response Time</div>
                        <div class="metric-value">${metricsData.time_response_seconds.toFixed(2)}s</div>
                        <div class="metric-description">
                            Total time from vehicle detection to complete median shift.
                        </div>
                    </div>
                    
                    <div class="metric-card">
                        <div class="metric-icon"><i class="fas fa-bullseye"></i></div>
                        <div class="metric-title">YOLO Detection Accuracy</div>
                        <div class="metric-value">${metricsData.yolo_accuracy_percent}%</div>
                        <div class="metric-description">
                            Real-world AI vehicle detection accuracy.
                            Based on YOLOv8 performance: 94% detection × 98% counting × 99% threshold logic.
                        </div>
                    </div>
                    
                    <div class="metric-card">
                        <div class="metric-icon"><i class="fas fa-car"></i></div>
                        <div class="metric-title">Trip Time Improvement</div>
                        <div class="metric-value">${metricsData.trip_time_improvement_percent.toFixed(1)}%</div>
                        <div class="metric-description">
                            Travel time reduction using BPR function.
                            Baseline: ${metricsData.trip_time_baseline_minutes?.toFixed(2) || (metricsData.trip_time_baseline_seconds / 60).toFixed(2)} min →
                            Improved: ${metricsData.trip_time_improved_minutes?.toFixed(2) || (metricsData.trip_time_improved_seconds / 60).toFixed(2)} min
                        </div>
                        <div class="metric-improvement">${metricsData.trip_time_saved_minutes?.toFixed(2) || (metricsData.trip_time_saved_seconds / 60).toFixed(2)} minutes saved per trip</div>
                    </div>
                    

                </div>
                
                <!-- Charts Section -->
                <div class="charts-section">
                    <div class="chart-container">
                        <div class="chart-title"><i class="fas fa-chart-bar"></i> System Performance Comparison</div>
                        <canvas id="comparisonChart" class="chart-canvas"></canvas>
                    </div>
                    
                    ${historyData.length > 1 ? `
                    <div class="chart-container">
                        <div class="chart-title"><i class="fas fa-chart-line"></i> All Simulations Comparison</div>
                        <canvas id="trendsChart" class="chart-canvas"></canvas>
                    </div>
                    ` : ''}
                </div>
                
                <!-- Detailed Comparison -->
                <div class="comparison-section">
                    <div class="comparison-card">
                        <div class="comparison-title"><i class="fas fa-road"></i> Baseline (3-3 Lanes)</div>
                        <div class="comparison-item">
                            <span class="comparison-label">Trip Time:</span>
                            <span class="comparison-value">${(metricsData.trip_time_baseline_minutes || metricsData.trip_time_baseline_seconds / 60).toFixed(2)} min</span>
                        </div>
                        <div class="comparison-item">
                            <span class="comparison-label">Capacity:</span>
                            <span class="comparison-value">120 veh/h</span>
                        </div>
                        <div class="comparison-item">
                            <span class="comparison-label">System Response:</span>
                            <span class="comparison-value">N/A</span>
                        </div>
                    </div>
                    
                    <div class="comparison-card">
                        <div class="comparison-title"><i class="fas fa-rocket"></i> Dynamic System (4-2 Lanes)</div>
                        <div class="comparison-item">
                            <span class="comparison-label">Trip Time:</span>
                            <span class="comparison-value">${(metricsData.trip_time_improved_minutes || metricsData.trip_time_improved_seconds / 60).toFixed(2)} min</span>
                        </div>
                        <div class="comparison-item">
                            <span class="comparison-label">Capacity:</span>
                            <span class="comparison-value">160 veh/h</span>
                        </div>
                        <div class="comparison-item">
                            <span class="comparison-label">System Response:</span>
                            <span class="comparison-value">${metricsData.time_response_seconds.toFixed(2)}s</span>
                        </div>
                    </div>
                </div>
                
                <!-- History Table -->
                ${historyData.length > 1 ? `
                <div class="history-section">
                    <div class="chart-container">
                        <div class="chart-title"><i class="fas fa-history"></i> Simulation History (${historyData.length} runs)</div>
                        <table class="history-table">
                            <thead>
                                <tr>
                                    <th>Session ID</th>
                                    <th>Date</th>
                                    <th>Trip Time Improvement</th>
                                    <th>Time Response</th>
                                </tr>
                            </thead>
                            <tbody>
                                ${historyData.slice().reverse().slice(0, 10).map(item => `
                                    <tr>
                                        <td>${item.session_id || 'N/A'}</td>
                                        <td>${item.timestamp ? new Date(item.timestamp).toLocaleString() : 'N/A'}</td>
                                        <td>${item.metrics?.trip_time_improvement_percent?.toFixed(2) || 'N/A'}%</td>
                                        <td>${item.metrics?.time_response_seconds?.toFixed(2) || 'N/A'}s</td>
                                    </tr>
                                `).join('')}
                            </tbody>
                        </table>
                    </div>
                </div>
                ` : ''}
            `;

    // Render charts
    setTimeout(() => {
        renderComparisonChart();
        if (historyData.length > 1) {
            renderTrendsChart();
        }
    }, 100);
}

function renderComparisonChart() {
    const ctx = document.getElementById('comparisonChart');
    if (!ctx) return;

    if (charts.comparison) {
        charts.comparison.destroy();
    }

    charts.comparison = new Chart(ctx, {
        type: 'bar',
        data: {
            labels: ['Trip Time (minutes)', 'System Response (seconds)'],
            datasets: [{
                label: 'Baseline (3-3 Lanes)',
                data: [
                    metricsData.trip_time_baseline_minutes || metricsData.trip_time_baseline_seconds / 60,
                    null
                ],
                backgroundColor: 'rgba(255, 100, 100, 0.7)',
                borderColor: 'rgba(255, 100, 100, 1)',
                borderWidth: 2
            }, {
                label: 'Dynamic System (4-2 Lanes)',
                data: [
                    metricsData.trip_time_improved_minutes || metricsData.trip_time_improved_seconds / 60,
                    metricsData.time_response_seconds
                ],
                backgroundColor: 'rgba(78, 204, 163, 0.7)',
                borderColor: 'rgba(78, 204, 163, 1)',
                borderWidth: 2
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    labels: { color: '#eee', font: { size: 14 } }
                },
                tooltip: {
                    callbacks: {
                        label: function (context) {
                            if (context.parsed.y === null) return '';
                            let label = context.dataset.label || '';
                            if (context.dataIndex === 0) {
                                return label + ': ' + context.parsed.y.toFixed(2) + ' min';
                            } else {
                                return label + ': ' + context.parsed.y.toFixed(2) + 's';
                            }
                        }
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: true,
                    ticks: { color: '#eee' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                },
                x: {
                    ticks: { color: '#eee' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                }
            }
        }
    });
}

function renderCO2Chart() {
    const ctx = document.getElementById('co2Chart');
    if (!ctx) return;

    if (charts.co2) {
        charts.co2.destroy();
    }

    charts.co2 = new Chart(ctx, {
        type: 'doughnut',
        data: {
            labels: ['Baseline Emissions', 'Emissions Saved'],
            datasets: [{
                data: [
                    metricsData.co2_improved_kg,
                    metricsData.co2_saved_kg
                ],
                backgroundColor: [
                    'rgba(255, 100, 100, 0.7)',
                    'rgba(78, 204, 163, 0.7)'
                ],
                borderColor: [
                    'rgba(255, 100, 100, 1)',
                    'rgba(78, 204, 163, 1)'
                ],
                borderWidth: 2
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'bottom',
                    labels: { color: '#eee', font: { size: 14 }, padding: 20 }
                },
                tooltip: {
                    callbacks: {
                        label: function (context) {
                            return context.label + ': ' + context.parsed.toFixed(4) + ' kg';
                        }
                    }
                }
            }
        }
    });
}

function renderTrendsChart() {
    const ctx = document.getElementById('trendsChart');
    if (!ctx) return;

    if (charts.trends) {
        charts.trends.destroy();
    }

    const labels = historyData.map((item, index) => {
        const date = new Date(item.timestamp);
        return date.toLocaleDateString() + ' ' + date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    });

    // Convert seconds to minutes
    const tripTimesBaseline = historyData.map(item => {
        const seconds = item.metrics?.trip_time_baseline_seconds || 0;
        const minutes = item.metrics?.trip_time_baseline_minutes;
        return minutes !== undefined ? minutes : (seconds / 60);
    });

    const tripTimesImproved = historyData.map(item => {
        const seconds = item.metrics?.trip_time_improved_seconds || 0;
        const minutes = item.metrics?.trip_time_improved_minutes;
        return minutes !== undefined ? minutes : (seconds / 60);
    });

    charts.trends = new Chart(ctx, {
        type: 'bar',
        data: {
            labels: labels,
            datasets: [{
                label: 'Trip Time Baseline (min)',
                data: tripTimesBaseline,
                backgroundColor: 'rgba(255, 100, 100, 0.7)',
                borderColor: 'rgba(255, 100, 100, 1)',
                borderWidth: 2
            }, {
                label: 'Trip Time Improved (min)',
                data: tripTimesImproved,
                backgroundColor: 'rgba(78, 204, 163, 0.7)',
                borderColor: 'rgba(78, 204, 163, 1)',
                borderWidth: 2
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    labels: { color: '#eee', font: { size: 14 } }
                },
                tooltip: {
                    callbacks: {
                        label: function (context) {
                            let label = context.dataset.label || '';
                            return label + ': ' + context.parsed.y.toFixed(2) + ' min';
                        }
                    }
                },
                datalabels: {
                    display: true,
                    color: '#fff',
                    font: {
                        weight: 'bold',
                        size: 12
                    },
                    formatter: function (value, context) {
                        return value.toFixed(2);
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: true,
                    title: {
                        display: true,
                        text: 'Trip Time (minutes)',
                        color: '#eee',
                        font: { size: 14 }
                    },
                    ticks: { color: '#eee' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                },
                x: {
                    ticks: {
                        color: '#eee',
                        maxRotation: 45,
                        minRotation: 45
                    },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                }
            }
        }
    });
}