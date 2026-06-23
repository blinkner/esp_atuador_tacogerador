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
    let mensagem = message.toString().split(',');
    console.log(mensagem);

    if (Ygraph.data.labels.length >= 500) {
        removeData(Ygraph);
        removeData(Ugraph);
    } 

    for (i = 0; i < 50; i++) {
        addData(Ygraph, mensagem[0 + 3*i], mensagem[1 + 3*i]);
        addData(Ugraph, mensagem[0 + 3*i], mensagem[2 + 3*i]);
        dados += mensagem[0 + 3*i] + ";" + mensagem[1 + 3*i] + ";" + mensagem[2 + 3*i] + ";" + mensagem[2 + 3*i] + "\n";
    }
});

function SalvarArquivo() {
    const blob = new Blob([dados], {type: "text/plain; charset=utf-8"});
    saveAs(blob, "dados.txt");
}