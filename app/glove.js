const DEVICE_NAME = 'SmartGlove';

const supportedServices = new Map([
	[BluetoothUUID.canonicalUUID('0x181A'), 'Environmental Sensor']
]);

const supportedCharacteristics = new Map([
	[BluetoothUUID.canonicalUUID('0x2A6D'), 'Pressure'],
	[BluetoothUUID.canonicalUUID('0x2A6E'), 'Temperature'],
	[BluetoothUUID.canonicalUUID('0x2A6F'), 'Humidity'],
	['b52338a6-b7fa-47d9-8db4-dbb86ac6b05c', 'Index of air quality (IAQ)'],
	['0d1ab684-14a4-479b-9dcd-86b6fc2e99fa', 'Static index of air quality (SIAQ)']
]);

const statusBox = document.getElementById('statusBox');
const statusMessage = document.getElementById('statusMessage');
const txtStatus = document.getElementById('txtStatus');
const bttnConnect = document.getElementById('bttnConnect');
const servicesArea = document.getElementById('servicesArea');

let bleServer;

bttnConnect.addEventListener('click', bluetoothConnect);

function statusReady(message = '') {
	statusBox.className = 'status ready';
	statusMessage.innerHTML = message;
}

function statusOk(message = '') {
	statusBox.className = 'status ok';
	statusMessage.innerHTML = message;
}

function statusFail(message = '') {
	statusBox.className = 'status fail';
	statusMessage.innerHTML = message;
}

function bluetoothConnect() {
	if (bleServer && bleServer.connected) {
		console.log('Already connected');
		return;
	}

	if (!navigator.bluetooth) {
		statusFail('This browser does not support Web Bluetooth. Please try another browser.');
		return;
	}

	bttnConnect.disabled = true;
	bttnConnect.className = 'disabled';
	
	console.log('Opening Bluetooth connection...');
	navigator.bluetooth.requestDevice({
		filters: [{name: DEVICE_NAME}],
		optionalServices: [getSupportedUuids()]
	})
	.then(device => {
		console.log('Connected to: ', device.name);
		statusOk('Connected to ' + device.name);
		device.addEventListener('gattservicedisconnected', onDisconnect);
		return device.gatt.connect();
	})
	.then(gattServer => {
		bleServer = gattServer;
		console.log('GATT server connected');
		return bleServer.getPrimaryServices();
	})
	.then(services => initServices(services))
	.catch(error => {
		if (bleServer && bleServer.connected) {
			bleServer.disconnect();
		}

		console.log('Error connecting to Bluetooth device: ', error);
		statusFail('Error connecting to Bluetooth device');
		bttnConnect.disabled = false;
		bttnConnect.className = '';
	})
}

function onDisconnect(event) {
	console.log('Disconnected from: ', event.target.device.name);
	statusReady('Disconnected');
	servicesArea.replaceChildren();
	bttnConnect.disabled = false;
	bttnConnect.className = '';
}

function getSupportedUuids() {
	const keys = supportedServices.keys();
	return Array.from(keys);
}

function initServices(services) {
	for (const service of services) {
		if (supportedServices.has(service.uuid)) {
			initService(service);
		} else {
			console.log('Found unsupported service with UUID "', service.uuid, '"');
		}
	}
}

function initService(service) {
	const serviceName = supportedServices.get(service.uuid);
	console.log('Found service "', serviceName, '" with UUID "', service.uuid, '"');

	let serviceDiv = document.createElement('div');
	serviceDiv.id = service.uuid;
	serviceDiv.className = 'serviceBox';
	servicesArea.appendChild(serviceDiv);

	let table = document.createElement('table');
	serviceDiv.appendChild(table);

	let titleRow = document.createElement('tr');
	table.appendChild(titleRow);

	let titleCell = document.createElement('td');
	titleCell.colSpan = 2;
	titleCell.className = 'serviceName';
	titleCell.innerHTML = serviceName;
	titleRow.appendChild(titleCell);

	service.getCharacteristics().then(characteristics => {
		initCharacteristics(serviceDiv, characteristics);
	})
}

function initCharacteristics(table, characteristics) {
	for (const characteristic of characteristics) {
		if (supportedCharacteristics.has(characteristic.uuid)) {
			initCharacteristic(table, characteristic);
		} else {
			console.log('Found unsupported characteristic with UUID "', characteristic.uuid, '"');
		}
	}
}

function initCharacteristic(table, characteristic) {
	const characteristicName = supportedCharacteristics.get(characteristic.uuid);
	console.log('Found characteristic "', characteristicName, '" with UUID "', characteristic.uuid, '"');

	let row = document.createElement('tr');
	table.appendChild(row);

	let nameCell = document.createElement('td');
	nameCell.innerHTML = characteristicName + ': ';
	row.appendChild(nameCell);

	let valCell = document.createElement('td');
	valCell.id = characteristic.uuid;
	valCell.innerHTML = 'No data';
	row.appendChild(valCell);

	characteristic.addEventListener('characteristicvaluechanged', onNotify);
	characteristic.startNotifications();
	console.log('Started notifications for UUID "', characteristic.uuid, '"');
}

function onNotify(event) {
	const valCell = document.getElementById(event.target.uuid);
	if (!valCell) {
		console.log('No target cell found for UUID "', event.target.uuid, '"');
	} else {
		const val = event.target.value.getFloat32(0, true); //Little endian
		valCell.innerHTML = val.toFixed(2);
	}
}
