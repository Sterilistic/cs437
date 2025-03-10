import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

class ChatPage extends StatefulWidget {
  final BluetoothDevice server;
  const ChatPage({required this.server});

  @override
  _ChatPage createState() => _ChatPage();
}

class _Message {
  int whom;
  String text;
  _Message(this.whom, this.text);
}

class _ChatPage extends State<ChatPage> {
  static final clientID = 0;
  BluetoothConnection? connection;
  List<_Message> messages = [];
  bool isConnecting = true;
  bool get isConnected => connection != null && connection!.isConnected;

  final TextEditingController textEditingController = TextEditingController();
  final ScrollController listScrollController = ScrollController();

  @override
  void initState() {
    super.initState();
    _connectToDevice();
  }

  @override
  void dispose() {
    if (isConnected) {
      connection?.dispose();
    }
    super.dispose();
  }

  Future<void> _connectToDevice() async {
    try {
      print('Connecting to device...');
      connection = await BluetoothConnection.toAddress(widget.server.address);
      print('Connected to the device');

      connection!.input!.listen((Uint8List data) {
        print('Received data: ${ascii.decode(data)}');
        setState(() {
          messages.add(_Message(1, ascii.decode(data)));
        });
      }).onDone(() {
        print('Disconnected by remote request');
        setState(() {
          isConnecting = false;
        });
      });

      setState(() {
        isConnecting = false;
      });

    } catch (e) {
      print('Error connecting to device: $e');
      setState(() {
        isConnecting = false;
      });
    }
  }

  void _sendMessage(String text) async {
    text = text.trim();
    textEditingController.clear();

    if (text.length > 0 && isConnected) {
      try {
        connection!.output.add(Uint8List.fromList(utf8.encode(text + "\n")));
        await connection!.output.allSent;
        print('Message sent successfully');

        setState(() {
          messages.add(_Message(clientID, text));
        });

        listScrollController.animateTo(
          listScrollController.position.maxScrollExtent,
          duration: Duration(milliseconds: 333),
          curve: Curves.easeOut,
        );
      } catch (e) {
        print('Error sending message: $e');
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final List<Row> list = messages.map((_message) {
      return Row(
        children: <Widget>[
          Container(
            child: Text(
              (_message.text.trim()),
              style: TextStyle(color: Colors.white),
            ),
            padding: EdgeInsets.all(12.0),
            margin: EdgeInsets.only(bottom: 8.0, left: 8.0, right: 8.0),
            width: 222.0,
            decoration: BoxDecoration(
              color: _message.whom == clientID ? Colors.blueAccent : Colors.grey,
              borderRadius: BorderRadius.circular(7.0),
            ),
          ),
        ],
        mainAxisAlignment: _message.whom == clientID
            ? MainAxisAlignment.end
            : MainAxisAlignment.start,
      );
    }).toList();

    return Scaffold(
      appBar: AppBar(
        title: Text(
          isConnecting
              ? 'Connecting to ${widget.server.name}...'
              : isConnected
                  ? 'Connected to ${widget.server.name}'
                  : 'Disconnected from ${widget.server.name}',
        ),
      ),
      body: SafeArea(
        child: Column(
          children: <Widget>[
            Flexible(
              child: ListView(
                padding: const EdgeInsets.all(12.0),
                controller: listScrollController,
                children: list,
              ),
            ),
            Row(
              children: <Widget>[
                Flexible(
                  child: Container(
                    margin: const EdgeInsets.only(left: 16.0),
                    child: TextField(
                      style: const TextStyle(fontSize: 15.0),
                      controller: textEditingController,
                      decoration: InputDecoration.collapsed(
                        hintText: isConnecting
                            ? 'Wait until connected...'
                            : isConnected
                                ? 'Type your message...'
                                : 'Chat got disconnected',
                        hintStyle: const TextStyle(color: Colors.grey),
                      ),
                      enabled: isConnected,
                    ),
                  ),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  child: IconButton(
                    icon: const Icon(Icons.send),
                    onPressed: isConnected
                        ? () => _sendMessage(textEditingController.text)
                        : null,
                  ),
                ),
              ],
            )
          ],
        ),
      ),
    );
  }
}
