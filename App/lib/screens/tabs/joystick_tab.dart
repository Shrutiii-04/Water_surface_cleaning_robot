import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import '/services/webs_service.dart';
import 'dart:async';


class JoystickTab extends StatefulWidget {
  final String robotId;
  const JoystickTab({super.key, required this.robotId});

  @override
  State<JoystickTab> createState() => _JoystickTabState();
}

class _JoystickTabState extends State<JoystickTab> {
  double x = 0;
  double y = 0;

  bool isConnected = false;

  @override
  void initState() {
    super.initState();

    // Just connect; WebsService handles auth_app internally
    WebsService.connect(widget.robotId);

    // Very simple polling to reflect connection state
    // (optional but helpful)
    Timer.periodic(const Duration(seconds: 1), (t) {
      if (!mounted) {
        t.cancel();
        return;
      }
      setState(() {
        isConnected = WebsService.isConnected;
      });
    });
  }

  @override
  void dispose() {
    WebsService.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF0A1433),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text(
              "Manual Control",
              style: TextStyle(color: Colors.white, fontSize: 22),
            ),

            const SizedBox(height: 20),

            Text(
              isConnected ? "Connected" : "Connecting...",
              style: TextStyle(
                color: isConnected ? Colors.green : Colors.red,
                fontSize: 16,
              ),
            ),

            const SizedBox(height: 40),

            Joystick(
              mode: JoystickMode.all,
              listener: (details) {
                setState(() {
                  x = details.x;
                  y = details.y;
                });

                if (!WebsService.isConnected) return;

                // ✅ send joystick to backend
                WebsService.sendJoystick(x, y);
              },
            ),

            const SizedBox(height: 40),

            Text("X: ${x.toStringAsFixed(2)}",
                style: const TextStyle(color: Colors.white)),
            Text("Y: ${y.toStringAsFixed(2)}",
                style: const TextStyle(color: Colors.white)),
          ],
        ),
      ),
    );
  }
}
