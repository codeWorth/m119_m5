const uuid_service = "5101";
const uuid_value_forward_x = "2101";
const uuid_value_forward_y = "2102";
const uuid_value_forward_z = "2103";
let connected_1 = false;

const uuid_service_2 = "5102";
const uuid_value_az = "2101";

let data = undefined;
let forward_x = undefined;
let forward_y = undefined;
let forward_z = undefined;
let lastData = undefined;

let az = undefined;

const noble = require('@abandonware/noble');

noble.on('stateChange', async (state) => {
  if (state === 'poweredOn') {
    console.log("start scanning")
    noble.startScanningAsync([uuid_service], false);
  }
});

noble.on('discover', async (peripheral) => {
  if (peripheral.advertisement.localName != "Andrew Arduino") {
    console.log("Ignoring", peripheral.advertisement.localName);
    return
  } else {
    console.log("Connecting to", peripheral.advertisement.localName);
  }

  await noble.stopScanningAsync();
  await peripheral.connectAsync();
  if (!connected_1) {
    const {characteristics} = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
      [uuid_service], 
      [uuid_value_forward_x, uuid_value_forward_y, uuid_value_forward_z]
    );

    const forward_x_char = characteristics.find(c => c.uuid == uuid_value_forward_x);
    const forward_y_char = characteristics.find(c => c.uuid == uuid_value_forward_y);
    const forward_z_char = characteristics.find(c => c.uuid == uuid_value_forward_z);

    forward_x_char.subscribe();
    forward_y_char.subscribe();
    forward_z_char.subscribe();

    forward_x_char.on('data', (data) => {
      forward_x = data.readFloatLE();
      storeData();
    });
    forward_y_char.on('data', (data) => {
      forward_y = data.readFloatLE();
      storeData();
    });
    forward_z_char.on('data', (data) => {
      forward_z = data.readFloatLE();
      storeData();
    });

    console.log("Connected!");
    connected_1 = true;
    noble.startScanningAsync([uuid_service_2], false);
  } else {
    const {characteristics} = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
      [uuid_service_2], 
      [uuid_value_az]
    );

    const forward_z_char = characteristics.find(c => c.uuid == uuid_value_az);
    forward_z_char.on('data', (data) => {
      az = data.readFloatLE();
      storeData();
    });
  }
});

function storeData() {
  if (forward_x === undefined || forward_y === undefined || forward_z === undefined || az === undefined) {
    return;
  }

  if (lastData !== undefined) {
    const dt = Date.now() - lastData;
    if (dt > 100) {
      console.log(dt, "since data!");
    }
  }
  lastData = Date.now();
  data = {
    x: forward_x,
    y: forward_y,
    z: forward_z,
    az: az
  };
  forward_x = undefined;
  forward_y = undefined;
  forward_z = undefined;
  az = undefined;
}

const WebSocket = require("ws");
const wss = new WebSocket.Server({ port: 3001 });

const sendData = (ws) => {
  return () => {
    if (data === undefined) return;
    ws.send(JSON.stringify(data));
    data = undefined;
  };
}

wss.on('connection', function connection(ws) {
  setInterval(sendData(ws), 30);
});

const fs = require('fs');
const http = require('http');

http.createServer(function (req, res) {
  fs.readFile(__dirname + req.url, function (err,data) {
    if (err) {
      res.writeHead(404);
      res.end(JSON.stringify(err));
      return;
    }
    res.writeHead(200);
    res.end(data);
  });
}).listen(3000);