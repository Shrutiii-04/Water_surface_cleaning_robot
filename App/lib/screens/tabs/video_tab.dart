import 'package:flutter/material.dart';

class VideoTab extends StatelessWidget {
  final String robotId;
  const VideoTab({super.key, required this.robotId});
  @override
  Widget build(BuildContext context) {
    return const Center(child: Text('Live video stream will appear here', style: TextStyle(color: Colors.white70)));
  }
}
