import 'package:flutter/material.dart';
import '../../services/mqtt_service.dart';

class PerformanceTab extends StatefulWidget {
  final String robotId;

  const PerformanceTab({super.key, required this.robotId});

  @override
  State<PerformanceTab> createState() => _PerformanceTabState();
}

class _PerformanceTabState extends State<PerformanceTab> {
  double batteryLevel = 0;
  double dustbinLevel = 0;
  double speed = 0;
  double distance = 0;

  @override
  void initState() {
    super.initState();

    // Listen for telemetry updates
    MqttService.telemetryCallback = (data) {
      if (!mounted) return;

      setState(() {
        batteryLevel = (data["battery"] ?? batteryLevel).toDouble();
        dustbinLevel = (data["dustbin"] ?? dustbinLevel).toDouble();
        speed = (data["speed"] ?? speed).toDouble();
        distance = (data["distance"] ?? distance).toDouble();
      });
    };
  }

  @override
  void dispose() {
    // IMPORTANT: Remove callback when leaving tab
    MqttService.telemetryCallback = null;
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Performance"),
        centerTitle: true,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: GridView.count(
          crossAxisCount: 2,
          mainAxisSpacing: 16,
          crossAxisSpacing: 16,
          children: [
            _metricCard(
              title: "Battery",
              icon: Icons.battery_full,
              value: "${batteryLevel.toInt()}%",
              progress: batteryLevel / 100,
              color: Colors.green,
            ),
            _metricCard(
              title: "Dustbin",
              icon: Icons.delete,
              value: "${dustbinLevel.toInt()}%",
              progress: dustbinLevel / 100,
              color: Colors.orange,
            ),
            _simpleCard(
              title: "Speed",
              icon: Icons.speed,
              value: "$speed m/s",
              color: Colors.blue,
            ),
            _simpleCard(
              title: "Distance",
              icon: Icons.route,
              value: "$distance m",
              color: Colors.purple,
            ),
          ],
        ),
      ),
    );
  }

  Widget _metricCard({
    required String title,
    required IconData icon,
    required String value,
    required double progress,
    required Color color,
  }) {
    return Card(
      elevation: 6,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Icon(icon, color: color, size: 36),
            Text(title, style: const TextStyle(fontWeight: FontWeight.bold)),
            Text(value, style: const TextStyle(fontSize: 20)),
            LinearProgressIndicator(
              value: progress,
              color: color,
              backgroundColor: color.withOpacity(0.2),
            ),
          ],
        ),
      ),
    );
  }

  Widget _simpleCard({
    required String title,
    required IconData icon,
    required String value,
    required Color color,
  }) {
    return Card(
      elevation: 6,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, size: 40, color: color),
          const SizedBox(height: 10),
          Text(title, style: const TextStyle(fontWeight: FontWeight.bold)),
          const SizedBox(height: 5),
          Text(
            value,
            style: const TextStyle(fontSize: 18),
          ),
        ],
      ),
    );
  }
}
