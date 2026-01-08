import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';

class RobotTrackingTab extends StatefulWidget {
  final String robotId;
  const RobotTrackingTab({super.key, required this.robotId});

  @override
  State<RobotTrackingTab> createState() => _RobotTrackingTabState();
}

class _RobotTrackingTabState extends State<RobotTrackingTab> {
  GoogleMapController? _mapController;

  // Robot simulated position
  LatLng _robotPosition = const LatLng(22.92, 82.86);
  final List<LatLng> _robotPath = [];

  // Robot status
  String _robotStatus = "Idle";
  Timer? _timer;
  final Random _random = Random();

  @override
  void initState() {
    super.initState();
   // _startFakeMovement();
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  void _startFakeMovement() {
    _timer = Timer.periodic(const Duration(seconds: 3), (_) {
      final dLat = (_random.nextDouble() - 0.5) * 0.001;
      final dLng = (_random.nextDouble() - 0.5) * 0.001;
      _robotPosition = LatLng(
        _robotPosition.latitude + dLat,
        _robotPosition.longitude + dLng,
      );
      _robotPath.add(_robotPosition);

      setState(() {
        _robotStatus = "Moving";
      });
    });
  }

  void _onMapCreated(GoogleMapController controller) {
    _mapController = controller;
  }

  Set<Marker> _buildMarkers() {
    return {
      Marker(
        markerId: const MarkerId("robot"),
        position: _robotPosition,
        infoWindow: const InfoWindow(title: "Robot"),
        icon: BitmapDescriptor.defaultMarkerWithHue(
          BitmapDescriptor.hueOrange,
        ),
      ),
    };
  }

  Set<Polyline> _buildPathLine() {
    return {
      Polyline(
        polylineId: const PolylineId("path"),
        points: _robotPath,
        width: 4,
        color: Colors.blue,
      ),
    };
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [

          // MAP
          GoogleMap(
            initialCameraPosition: const CameraPosition(
              target: LatLng(22.92, 82.86),
              zoom: 16,
            ),
            onMapCreated: _onMapCreated,
            markers: _buildMarkers(),
            polylines: _buildPathLine(),
            myLocationEnabled: true,
            mapType: MapType.hybrid,
          ),

          // STATUS CARD
          Positioned(
            bottom: 12,
            left: 12,
            right: 12,
            child: Card(
              elevation: 4,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: Row(
                  children: [
                    const Icon(Icons.smart_toy),
                    const SizedBox(width: 10),
                    const Text(
                      "Robot Status: ",
                      style: TextStyle(fontWeight: FontWeight.bold),
                    ),
                    Text(_robotStatus),
                    const Spacer(),
                    IconButton(
                      icon: const Icon(Icons.stop),
                      onPressed: () {
                        _timer?.cancel();
                        setState(() => _robotStatus = "Stopped");
                      },
                    ),
                    IconButton(
                      icon: const Icon(Icons.play_arrow),
                      onPressed: () {
                        _startFakeMovement();
                        setState(() => _robotStatus = "Moving");
                      },
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
