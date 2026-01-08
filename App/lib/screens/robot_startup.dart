import 'package:flutter/material.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';

import 'robot_pairing.dart';
import 'mode_selection.dart';
import 'login_screen.dart';

class RobotStartup extends StatefulWidget {
  const RobotStartup({super.key});

  @override
  State<RobotStartup> createState() => _RobotStartupState();
}

class _RobotStartupState extends State<RobotStartup> {
  String? robotId;
  bool loading = true;

  @override
  void initState() {
    super.initState();
    _loadRobot();
  }

  Future<void> _loadRobot() async {
    final uid = FirebaseAuth.instance.currentUser!.uid;

    final doc = await FirebaseFirestore.instance
        .collection("users")
        .doc(uid)
        .get();

    if (doc.exists) {
      robotId = doc.data()?["robotId"];
    }

    setState(() => loading = false);
  }

  // Sign Out Function
  Future<void> _confirmSignOut() async {
    final shouldSignOut = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Sign Out"),
        content: const Text("Are you sure you want to sign out?"),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text("Cancel"),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text("Sign Out"),
          ),
        ],
      ),
    );

    if (shouldSignOut == true) {
      await FirebaseAuth.instance.signOut();

      if (!mounted) return;

      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (_) => const LoginScreen()),
      );
    }
  }


  @override
  Widget build(BuildContext context) {
    if (loading) {
      return const Scaffold(
        body: Center(child: CircularProgressIndicator()),
      );
    }

    // No robot yet, go to pairing
    if (robotId == null) {
      return const RobotPairing();
    }

    // Robot found, show connect screen
    return Scaffold(
      appBar: AppBar(
        title: const Text("Connect to Robot"),
        actions: [
          IconButton(
            icon: const Icon(Icons.logout),
            onPressed: _confirmSignOut,
          )
        ],
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              "Robot Found:\n$robotId",
              textAlign: TextAlign.center,
              style: const TextStyle(fontSize: 22),
            ),
            const SizedBox(height: 20),

            ElevatedButton(
              onPressed: () {
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(
                    builder: (_) => ModeSelection(robotId: robotId!),
                  ),
                );
              },
              child: const Text("Connect to Robot"),
            ),

            const SizedBox(height: 30),

            TextButton(
              onPressed: () {
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(
                    builder: (_) => const RobotPairing(),
                  ),
                );
              },
              child: const Text("Pair New Robot"),
            ),
          ],
        ),
      ),
    );
  }
}
