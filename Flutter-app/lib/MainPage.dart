import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import './ChatPage.dart';

class MainPage extends StatefulWidget {
  @override
  _MainPage createState() => _MainPage();
}

class _MainPage extends State<MainPage> {
  BluetoothDevice? _device;

  @override
  void initState() {
    super.initState();
    _fetchBondedDevices();
  }

  void _fetchBondedDevices() async {
    try {
      List<BluetoothDevice> devices = await FlutterBluetoothSerial.instance.getBondedDevices();
      print("Found ${devices.length} bonded devices");
      
      for (var device in devices) {
        print("Device found:");
        print("  Name: ${device.name}");
        print("  Address: ${device.address}");
      }

      setState(() {
        for (var device in devices) {
          if (device.name == "raspberrypi") {
            _device = device;
          }
        }
      });
    } catch (e) {
      print("Error while fetching devices: $e");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Chat Demo'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _fetchBondedDevices,
          ),
        ],
      ),
      body: ListView(
        children: <Widget>[
          Divider(),
          ListTile(
            title: Text("Device: " + (_device?.name ?? "No device selected")),
          ),
          ListTile(
            title: ElevatedButton(
              child: const Text('Start Chat'),
              onPressed: () {
                if (_device != null) {
                  print('Connect -> selected ' + _device!.address);
                  _startChat(context, _device!);
                } else {
                  print('Connect -> no device selected');
                }
              },
            ),
          ),
        ],
      ),
    );
  }

  void _startChat(BuildContext context, BluetoothDevice server) {
    Navigator.of(context).push(
      MaterialPageRoute(builder: (context) => ChatPage(server: server)),
    );
  }
}