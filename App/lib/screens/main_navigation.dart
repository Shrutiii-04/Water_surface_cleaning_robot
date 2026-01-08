import 'package:flutter/material.dart';

import 'tabs/home_tab.dart';
import 'tabs/video_tab.dart';
import 'tabs/performance_tab.dart';
import 'tabs/joystick_tab.dart';
import 'robot_tracking_tab.dart';
import '../services/mqtt_service.dart';

class MainNavigation extends StatefulWidget {
  final int initialIndex;
  final bool isManualMode;
  final String robotId;

  const MainNavigation({
    super.key,
    this.initialIndex = 0,
    required this.isManualMode,
    required this.robotId,
  });

  @override
  State<MainNavigation> createState() => _MainNavigationState();
}

class _MainNavigationState extends State<MainNavigation> {
  late int _idx;
  late List<Widget> pages;
  late List<BottomNavigationBarItem> navItems;

  @override
  void initState() {
    super.initState();

    _idx = widget.initialIndex;

    // Starts MQTT connection when MainNavigation loads
    Future.delayed(Duration.zero, () {
      MqttService.connect(widget.robotId);
    });

    // Configuring Screens Based On Mode
    if (widget.isManualMode) {
      // Manual Mode
      pages = [
        JoystickTab(robotId: widget.robotId),
        RobotTrackingTab(robotId: widget.robotId),
        VideoTab(robotId: widget.robotId),
        PerformanceTab(robotId: widget.robotId),
      ];

      navItems = const [
        BottomNavigationBarItem(icon: Icon(Icons.gamepad), label: 'Joystick'),
        BottomNavigationBarItem(icon: Icon(Icons.directions_boat), label: 'Track Path'),
        BottomNavigationBarItem(icon: Icon(Icons.videocam), label: 'Video'),
        BottomNavigationBarItem(icon: Icon(Icons.bar_chart), label: 'Performance'),
      ];
    } else {
      // Auto Mode
      pages = [
        HomeTab(robotId: widget.robotId),
        RobotTrackingTab(robotId: widget.robotId),
        VideoTab(robotId: widget.robotId),
        PerformanceTab(robotId: widget.robotId),
      ];

      navItems = const [
        BottomNavigationBarItem(icon: Icon(Icons.map), label: 'Home'),
        BottomNavigationBarItem(icon: Icon(Icons.directions_boat), label: 'Track Path'),
        BottomNavigationBarItem(icon: Icon(Icons.videocam), label: 'Video'),
        BottomNavigationBarItem(icon: Icon(Icons.bar_chart), label: 'Performance'),
      ];
    }

    if (_idx >= pages.length) _idx = 0;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF071127),
      appBar: AppBar(
        title: const Text('WSCR'),
        backgroundColor: const Color(0xFF071127),
      ),
      body: pages[_idx],
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _idx,
        onTap: (i) => setState(() => _idx = i),
        type: BottomNavigationBarType.fixed,
        backgroundColor: const Color(0xFF071127),
        selectedItemColor: const Color(0xFF00E5FF),
        unselectedItemColor: Colors.white54,
        items: navItems,
      ),
      drawer: Drawer(
        backgroundColor: const Color(0xFF071127),
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            const DrawerHeader(
              decoration: BoxDecoration(color: Color(0xFF0B2033)),
              child: Text(
                'WSCR',
                style: TextStyle(color: Colors.white, fontSize: 20),
              ),
            ),
            _drawerItem(Icons.person, "Profile"),
            _drawerItem(Icons.settings, "Settings"),
            _drawerItem(Icons.devices, "Connected Robots"),
            _drawerItem(Icons.analytics, "Analytics"),
            _drawerItem(Icons.notifications, "Notifications"),
            const Divider(color: Colors.white12),
            _drawerItem(Icons.logout, "Logout", onTap: () {
              Navigator.pushReplacementNamed(context, '/');
            }),
          ],
        ),
      ),
    );
  }

  Widget _drawerItem(IconData icon, String text, {VoidCallback? onTap}) {
    return ListTile(
      leading: Icon(icon, color: Colors.white70),
      title: Text(text, style: const TextStyle(color: Colors.white70)),
      onTap: onTap ?? () {},
    );
  }
}
