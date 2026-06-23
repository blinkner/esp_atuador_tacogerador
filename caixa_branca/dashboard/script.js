const {
    SciChartSurface,
    NumericAxis,
    FastLineRenderableSeries,
    XyDataSeries,
    NumberRange
} = SciChart;

let yChart;
let uChart;
let yData;
let uData;

let dados = "TIME;VOLTAGE;CONTROL_SIGNAL\n";
let t = 0;
let u = 0;
let y = 0;

async function init() {

    // Gráfico da saída
    const {
        sciChartSurface: ySurface,
        wasmContext: yWasm
    } = await SciChartSurface.create("y-chart");

    yChart = ySurface;

    yChart.xAxes.add(new NumericAxis(yWasm));
    yChart.yAxes.add(new NumericAxis(yWasm, { axisTitle: "Tensão (mV)"}));
    yChart.yAxes.get(0).visibleRange = new NumberRange(0, 3100);

    yData = new XyDataSeries(yWasm, {
        fifoCapacity: 10000
    });

    yChart.renderableSeries.add(
        new FastLineRenderableSeries(yWasm, {
            dataSeries: yData,
            stroke: "steelblue"
        })
    );

    // Gráfico do controle
    const {
        sciChartSurface: uSurface,
        wasmContext: uWasm
    } = await SciChartSurface.create("u-chart");

    uChart = uSurface;

    uChart.xAxes.add(new NumericAxis(uWasm, { axisTitle: "Tempo (ms)"}));
    uChart.yAxes.add(new NumericAxis(uWasm, { axisTitle: "Sinal de Controle"}));
    uChart.yAxes.get(0).visibleRange = new NumberRange(0, 100);

    uData = new XyDataSeries(uWasm, {
        fifoCapacity: 10000
    });

    uChart.renderableSeries.add(
        new FastLineRenderableSeries(uWasm, {
            dataSeries: uData,
            stroke: "firebrick"
        })
    );
}

await init();

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
    
    for (let i = 0; i < 50; i++) {
        t = mensagem[0 + 3*i];
        y = mensagem[1 + 3*i];
        u = mensagem[2 + 3*i];
        yData.append(t, y);
        uData.append(t, u);

        const range = new NumberRange(
            Math.max(0, t - 10000),
            t
        );

        yChart.xAxes.get(0).visibleRange = range;
        uChart.xAxes.get(0).visibleRange = range;

        dados += mensagem[0 + 3*i] + ";" + mensagem[1 + 3*i] + ";" + mensagem[2 + 3*i] + ";" + "0.0" + "\n";
    }
});

document.getElementById('salvar-dados').addEventListener('click', () => {
    const blob = new Blob([dados], {type: "text/plain; charset=utf-8"});
    saveAs(blob, "dados.txt");
});