import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:intl/intl.dart';
import 'package:cloud_firestore/cloud_firestore.dart';

import '../../services/mqtt_service.dart';

class HomeTab extends StatefulWidget {
  final String robotId;
  const HomeTab({super.key, required this.robotId});

  @override
  State<HomeTab> createState() => _HomeTabState();
}

class _HomeTabState extends State<HomeTab> {
  GoogleMapController? _mapController;

  List<LatLng> _polygonPoints = [];
  Set<Polygon> _polygons = {};
  bool _drawing = false;

  MapType _currentMapType = MapType.hybrid;

  void _onMapCreated(GoogleMapController controller) {
    _mapController = controller;
  }

  void _onMapTap(LatLng point) {
    if (!_drawing) return;

    setState(() {
      _polygonPoints.add(point);
      _polygons = {
        Polygon(
          polygonId: const PolygonId("cleaning_area"),
          points: _polygonPoints,
          fillColor: Colors.green.withOpacity(0.3),
          strokeColor: Colors.green,
          strokeWidth: 2,
        ),
      };
    });
  }

  void _undoLastPoint() {
    if (_polygonPoints.isEmpty) return;

    setState(() {
      _polygonPoints.removeLast();
      if (_polygonPoints.isEmpty) {
        _polygons.clear();
      } else {
        _polygons = {
          Polygon(
            polygonId: const PolygonId("cleaning_area"),
            points: _polygonPoints,
            fillColor: Colors.green.withOpacity(0.3),
            strokeColor: Colors.green,
            strokeWidth: 2,
          ),
        };
      }
    });
  }

  void _toggleDrawing() {
    setState(() => _drawing = !_drawing);
    if (!_drawing) _finishPolygon();
  }

  void _finishPolygon() {
    if (_polygonPoints.length < 3) return;

    ScaffoldMessenger.of(context)
        .showSnackBar(const SnackBar(content: Text("Area selected")));
  }

  // 🔥 SAVE MISSION TO FIRESTORE
  Future<void> _saveMissionToFirestore(
      String missionName, Map<String, dynamic> missionJson) async {
    final missionsRef = FirebaseFirestore.instance
        .collection("robot_private")
        .doc(widget.robotId)
        .collection("missions");

    final missionId = DateTime.now().millisecondsSinceEpoch.toString();

    await missionsRef.doc(missionId).set({
      "missionId": missionId,
      "missionName": missionName,
      "coordinates": _polygonPoints
          .map((p) => {"lat": p.latitude, "lng": p.longitude})
          .toList(),
      "status": "pending",
      "createdAt": FieldValue.serverTimestamp(),
    });
  }

  // 🔥 MISSION SEND WORKFLOW
  void _sendMissionToRobot() async {
    if (_polygonPoints.length < 3) {
      ScaffoldMessenger.of(context)
          .showSnackBar(const SnackBar(content: Text("Draw an area first")));
      return;
    }

    // Ask mission name
    final missionNameController = TextEditingController();
    final missionName = await showDialog<String>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text("Mission Name"),
        content: TextField(
          controller: missionNameController,
          decoration: const InputDecoration(hintText: "Enter mission name"),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, null),
            child: const Text("Cancel"),
          ),
          ElevatedButton(
            onPressed: () =>
                Navigator.pop(context, missionNameController.text.trim()),
            child: const Text("Save"),
          ),
        ],
      ),
    );

    if (missionName == null || missionName.isEmpty) return;

    // Close polygon
    final poly = List<LatLng>.from(_polygonPoints);
    if (poly.first != poly.last) poly.add(poly.first);

    // Create mission JSON
    final missionJson = {
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          poly.map((p) => [p.longitude, p.latitude]).toList(),
        ],
      }
    };

    // 1️⃣ Save to Firestore
    await _saveMissionToFirestore(missionName, missionJson);

    // 2️⃣ Send to Robot via MQTT
    MqttService.publishMission(widget.robotId, missionJson);

    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text("Mission '$missionName' sent to robot")),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(children: [
        GoogleMap(
          onMapCreated: _onMapCreated,
          onTap: _onMapTap,
          polygons: _polygons,
          mapType: _currentMapType,
          initialCameraPosition: const CameraPosition(
            target: LatLng(22.92, 82.86),
            zoom: 16,
          ),
          myLocationEnabled: true,
          myLocationButtonEnabled: true,
        ),

        // MAP TYPE SWITCH
        Positioned(
          top: 40,
          right: 12,
          child: Card(
            elevation: 4,
            child: PopupMenuButton<MapType>(
              icon: const Icon(Icons.layers),
              onSelected: (type) => setState(() => _currentMapType = type),
              itemBuilder: (_) => const [
                PopupMenuItem(value: MapType.normal, child: Text("Normal")),
                PopupMenuItem(value: MapType.satellite, child: Text("Satellite")),
                PopupMenuItem(value: MapType.hybrid, child: Text("Hybrid")),
                PopupMenuItem(value: MapType.terrain, child: Text("Terrain")),
              ],
            ),
          ),
        ),

        // BOTTOM BAR
        Positioned(
          bottom: 12,
          left: 12,
          right: 12,
          child: Card(
            child: Row(children: [
              const SizedBox(width: 8),
              Icon(Icons.map, color: _drawing ? Colors.red : Colors.green),
              const SizedBox(width: 6),
              Text(_drawing ? "Drawing..." : "Tap DRAW"),
              const Spacer(),

              IconButton(
                icon: const Icon(Icons.undo),
                onPressed: _drawing ? _undoLastPoint : null,
              ),

              IconButton(
                icon: const Icon(Icons.send),
                onPressed: _sendMissionToRobot,
              ),

              IconButton(
                icon: Icon(_drawing ? Icons.done : Icons.crop_square),
                onPressed: _toggleDrawing,
              ),
            ]),
          ),
        ),
      ]),
    );
  }
}
