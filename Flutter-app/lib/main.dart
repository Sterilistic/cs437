import 'package:flutter/material.dart';
import 'package:flutter_blue_app/MainPage.dart';


void main() => runApp(new ExampleApplication());

class ExampleApplication extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(home: MainPage());
  }
}