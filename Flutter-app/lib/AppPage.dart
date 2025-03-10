import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class AppPage extends StatefulWidget {
  final BluetoothDevice server;

  const AppPage({required this.server});

  @override
  _AppPage createState() => new _AppPage();
}

class _AppPage extends State<AppPage> {
  BluetoothCharacteristic? characteristic;
  BluetoothDevice? device;
  BluetoothService? service;

  String _messageBuffer = '';
  String message = "";

  bool isConnecting = true;
  bool get isConnected => characteristic != null;

  bool isDisconnecting = false;

  @override
  void initState() {
    super.initState();
    _connectToDevice();
  }

  @override
  void dispose() {
    if (isConnected) {
      isDisconnecting = true;
      characteristic!.value.listen((data) {}).cancel();
      characteristic = null;
      device!.disconnect();
      device = null;
    }
    super.dispose();
  }

  Future<void> _connectToDevice() async {
    try {
      await widget.server.connect();
      device = widget.server;

      List<BluetoothService> services = await device!.discoverServices();
      BluetoothService? targetService;
      BluetoothCharacteristic? targetCharacteristic;

      for (var s in services) {
        if (s.uuid.toString() == 'YOUR_SERVICE_UUID') {
          targetService = s;
          for (var c in s.characteristics) {
            if (c.uuid.toString() == 'YOUR_CHARACTERISTIC_UUID') {
              targetCharacteristic = c;
              break;
            }
          }
          break;
        }
      }

      if (targetService != null && targetCharacteristic != null) {
        setState(() {
          characteristic = targetCharacteristic;
          isConnecting = false;
          isDisconnecting = false;
        });

        characteristic!.value.listen(_onDataReceived);
      } else {
        print('Service or Characteristic not found.');
      }
    } catch (e) {
      print('Failed to connect: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    final Text command = Text(message);

    return Scaffold(
      appBar: AppBar(
        title: Text("Example App"),
      ),
      body: SafeArea(
        child: Column(
          children: <Widget>[
            command,
            ElevatedButton(
              child: const Text('A'),
              onPressed: () {
                _sendMessage("A");
              },
            ),
            ElevatedButton(
              child: const Text('B'),
              onPressed: () {
                _sendMessage("B");
              },
            ),
          ],
        ),
      ),
    );
  }

  void _onDataReceived(List<int> data) {
    String dataString = utf8.decode(data);
    setState(() {
      message = dataString;
    });
  }

  void _sendMessage(String text) async {
    text = text.trim();

    if (text.isNotEmpty && characteristic != null) {
      try {
        await characteristic!.write(utf8.encode(text));
      } catch (e) {
        print('Error sending message: $e');
        setState(() {});
      }
    }
  }
}
