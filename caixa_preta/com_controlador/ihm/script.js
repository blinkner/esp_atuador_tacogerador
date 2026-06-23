const yChart = document.getElementById('y-chart');
const uChart = document.getElementById('u-chart');

let dados = "TIME;VOLTAGE;CONTROL_SIGNAL;ERRO\n";

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
    chart.update('none');
}
function removeData(chart) {
    chart.data.labels.splice(0, 50);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.splice(0, 50);
    });
    chart.update('none');
}

const btn_referencia = document.getElementById('btn-referencia');
btn_referencia.addEventListener('click', () => {
    let referencia_value = document.getElementById('referencia').value;
    client.publish('planta/tacogerador/referencia', referencia_value);
});

const client = mqtt.connect('ws://localhost:9001');
client.on('connect', () => {
    console.log('Conectado ao broker MQTT');
    const topic = 'planta/tacogerador/voltage';
    client.subscribe(topic, (err) => {
        if (!err) {
            console.log('Inscrito no tópico: ' + topic);
        }
    });
});
client.on('message', (topic, message) => {
    // console.log('Received message: ' + message.toString() + ' on topic: ' + topic);
    // document.getElementById('data').textContent = message.toString();

    message = "[" + message + "]";
    message = message.replace(",]", "]")
    obj = JSON.parse(message);

    if (Ygraph.data.labels.length >= 500) {
        removeData(Ygraph);
        removeData(Ugraph);
    } 
    obj.forEach(item => {
        if (item.u != 0) {
            addData(Ygraph, item.t, item.y);
            addData(Ugraph, item.t, item.u);
            dados += item.t + ";" + item.y + ";" + item.u + ";" + item.e + "\n";
        }
    });
});

function SalvarArquivo() {
    const blob = new Blob([dados], {type: "text/plain; charset=utf-8"});
    saveAs(blob, "dados.txt");
}