const yChart = document.getElementById('y-chart');
const uChart = document.getElementById('u-chart');

Ygraph = new Chart(yChart, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Saída',
            data: [],
            borderWidth: 1,
            fill: false,
            borderColor: 'rgb(0, 0, 255)',
            tension: 0.1,
            pointStyle: false
        }]
    }
});
Ugraph = new Chart(uChart, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Sinal de Controle',
            data: [],
            borderWidth: 1,
            fill: false,
            borderColor: 'rgb(255, 0, 0)',
            tension: 0.1,
            pointStyle: false
        }]
    }
});

function addData(chart, label, newData) {
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(newData);
    });
    chart.update();
}

const btn_duty = document.getElementById('btn-duty');
btn_duty.addEventListener('click', () => {
    let duty_value = document.getElementById('duty').value;
    client.publish('planta/motor/pwm', duty_value);
});

const client = mqtt.connect('ws://localhost:9001');
client.on('connect', () => {
    console.log('Conectado ao broker MQTT');
    const topic = 'planta/tacogerador/voltage';
    client.subscribe(topic, (err) => {
        if (!err) {
            console.log('Subscribed to topic: ' + topic);
        }
    });
});
client.on('message', (topic, message) => {
    console.log('Received message: ' + message.toString() + ' on topic: ' + topic);
    // document.getElementById('data').textContent = message.toString();

    let obj = JSON.parse(message)
    addData(Ygraph, obj.time, obj.voltage);
    addData(Ugraph, obj.time, obj.control_signal);
});