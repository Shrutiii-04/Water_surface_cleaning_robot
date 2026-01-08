import 'package:flutter/material.dart';
import '../services/api_service.dart';
import 'mode_selection.dart';

class RobotPairing extends StatefulWidget {
  const RobotPairing({super.key});

  @override
  State<RobotPairing> createState() => _RobotPairingState();
}

class _RobotPairingState extends State<RobotPairing> {
  final _robotIdController = TextEditingController();
  final _pairCodeController = TextEditingController();
  bool isLoading = false;

  Future<void> connectRobot() async {
    final robotId = _robotIdController.text.trim();
    final pairCode = _pairCodeController.text.trim();

    if (robotId.isEmpty || pairCode.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text("Enter Robot ID & Pair Code")),
      );
      return;
    }

    setState(() => isLoading = true);

    try {
      await ApiService.pairRobot(robotId, pairCode);

      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text("Robot Paired Successfully")),
      );

      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (_) => ModeSelection(robotId: robotId)),
    );
    } catch (e) {
      ScaffoldMessenger.of(context)
          .showSnackBar(SnackBar(content: Text("Error: $e")));
    }

    setState(() => isLoading = false);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Robot Pairing"), centerTitle: true),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            TextField(
              controller: _robotIdController,
              decoration: const InputDecoration(
                labelText: "Robot ID",
                border: OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 15),
            TextField(
              controller: _pairCodeController,
              obscureText: true,
              decoration: const InputDecoration(
                labelText: "Pair Code",
                border: OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 20),

            // Submit Button
            SizedBox(
              width: double.infinity,
              height: 50,
              child: ElevatedButton(
                onPressed: isLoading ? null : connectRobot,
                child: isLoading
                    ? const CircularProgressIndicator(color: Colors.white)
                    : const Text("Connect Robot"),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
