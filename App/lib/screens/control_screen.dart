import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'main_navigation.dart';

class ControlScreen extends StatefulWidget {
  final String robotId;
  final bool isManualMode;

  const ControlScreen({
    super.key,
    required this.robotId,
    required this.isManualMode,
  });

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}


class _ControlScreenState extends State<ControlScreen> {
  double x = 0.0;
  double y = 0.0;
  int _idx = 0; // bottom navigation index

  void _onTabSelected(int i) {
    setState(() => _idx = i);

    Navigator.pushReplacement(
      context,
      MaterialPageRoute(
        builder: (_) => MainNavigation(
          initialIndex: i,
          isManualMode: widget.isManualMode, // required
          robotId: widget.robotId,           // required
        ),
      ),
    );
  }


  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF0A1433),
      appBar: AppBar(
        backgroundColor: const Color(0xFF0A1433),
        elevation: 0,
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: Colors.white),
          onPressed: () => Navigator.pop(context),
        ),
        centerTitle: true,
        title: const Text(
          "Manual Control",
          style: TextStyle(color: Colors.white),
        ),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text(
              "Joystick Control",
              style: TextStyle(color: Colors.white, fontSize: 20),
            ),
            const SizedBox(height: 30),
            Joystick(
              mode: JoystickMode.all,
              listener: (details) {
                setState(() {
                  x = details.x;
                  y = details.y;
                });
              },
            ),
            const SizedBox(height: 40),
            Text(
              "X: ${x.toStringAsFixed(2)}",
              style: const TextStyle(color: Colors.white, fontSize: 16),
            ),
            Text(
              "Y: ${y.toStringAsFixed(2)}",
              style: const TextStyle(color: Colors.white, fontSize: 16),
            ),
          ],
        ),
      ),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _idx,
        onTap: _onTabSelected,
        backgroundColor: const Color(0xFF071127),
        selectedItemColor: const Color(0xFF00E5FF),
        unselectedItemColor: Colors.white54,
        items: const [
          BottomNavigationBarItem(icon: Icon(Icons.map), label: 'Map'),
          BottomNavigationBarItem(icon: Icon(Icons.videocam), label: 'Video'),
          BottomNavigationBarItem(icon: Icon(Icons.bar_chart), label: 'Performance'),
          BottomNavigationBarItem(icon: Icon(Icons.battery_full), label: 'Dashboard'),
        ],
      ),
    );
  }
}
