const yChart = document.getElementById('y-chart');
const uChart = document.getElementById('u-chart');

let dados = "TIME\tADC_BITS\tVOLTAGE\tCONTROL_SIGNAL\n";

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
    },
    options: {
        scales: {
            y: {
                suggestedMin: 0,
                suggestedMax: 3100
            }
        }
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
    },
    options: {
        scales: {
            y: {
                suggestedMin: 0,
                suggestedMax: 100
            }
        }
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
    dados += obj.time + "\t" + obj.adc_bits + "\t" + obj.voltage + "\t" + obj.control_signal + "\n";
});

function SalvarArquivo() {
    const blob = new Blob([dados], {type: "text/plain; charset=utf-8"});
    saveAs(blob, "dados.txt");
}