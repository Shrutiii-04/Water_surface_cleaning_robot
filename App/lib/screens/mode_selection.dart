import 'package:flutter/material.dart';
import 'main_navigation.dart';

class ModeSelection extends StatefulWidget {
  final String robotId;

  const ModeSelection({
    super.key,
    required this.robotId,
  });

  @override
  State<ModeSelection> createState() => _ModeSelectionState();
}

class _ModeSelectionState extends State<ModeSelection> {
  bool _manual = true;

  void _goNext() {
    Navigator.pushReplacement(
      context,
      MaterialPageRoute(
        builder: (_) => MainNavigation(
          initialIndex: 0,
          isManualMode: _manual,
          robotId: widget.robotId,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF071127),
      appBar: AppBar(title: const Text('Select Mode')),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            const SizedBox(height: 12),
            const Text(
              'Choose operating mode',
              style: TextStyle(color: Colors.white, fontSize: 18),
            ),
            const SizedBox(height: 18),

            ToggleButtons(
              isSelected: [_manual, !_manual],
              onPressed: (i) => setState(() => _manual = i == 0),
              borderRadius: BorderRadius.circular(8),
              selectedColor: Colors.black,
              color: Colors.white,
              fillColor: const Color(0xFF00E5FF),
              children: const [
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 18, vertical: 10),
                  child: Text('Manual'),
                ),
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 18, vertical: 10),
                  child: Text('Automatic'),
                ),
              ],
            ),

            const SizedBox(height: 28),

            Card(
              color: Colors.white10,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12),
              ),
              child: Padding(
                padding: const EdgeInsets.all(14.0),
                child: Column(
                  children: [
                    const Text(
                      'Common Controls',
                      style: TextStyle(
                        color: Colors.white,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    const SizedBox(height: 12),

                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceAround,
                      children: [
                        ElevatedButton.icon(
                          onPressed: _goNext,
                          icon: const Icon(Icons.play_arrow),
                          label: const Text('Start'),
                        ),
                        ElevatedButton.icon(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.redAccent,
                          ),
                          onPressed: () {
                            ScaffoldMessenger.of(context).showSnackBar(
                              const SnackBar(
                                content: Text('Stop pressed (mock)'),
                              ),
                            );
                          },
                          icon: const Icon(Icons.stop),
                          label: const Text('Stop'),
                        ),
                        ElevatedButton.icon(
                          onPressed: () {
                            ScaffoldMessenger.of(context).showSnackBar(
                              const SnackBar(
                                content: Text('Power toggled (mock)'),
                              ),
                            );
                          },
                          icon: const Icon(Icons.power_settings_new),
                          label: const Text('Power'),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 18),

            Expanded(
              child: Center(
                child: Text(
                  _manual
                      ? 'Manual: joystick control'
                      : 'Automatic: Area selection & navigation',
                  style: const TextStyle(color: Colors.white70),
                  textAlign: TextAlign.center,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
