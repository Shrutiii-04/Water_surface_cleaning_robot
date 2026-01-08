import 'dart:convert';
import 'dart:async';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:firebase_auth/firebase_auth.dart';

class WebsService {
  static WebSocketChannel? _channel;
  static StreamSubscription? _sub;

  static String? _robotId;
  static bool _isConnected = false;

  // Callback
  static Function(Map<String, dynamic>)? onTelemetry;

  // Connect websocket
  static Future<void> connect(String robotId) async {
    _robotId = robotId;

    if (_isConnected) return;

    final firebaseUser = FirebaseAuth.instance.currentUser;
    if (firebaseUser == null) {
      print("NO Firebase user logged in");
      return;
    }

    final token = await firebaseUser.getIdToken();

    try {
      print("Connecting to WebSocket");

      _channel = WebSocketChannel.connect(
        Uri.parse("ws://51.21.101.122:9000"),
      );

      _isConnected = true;

      _channel!.sink.add(jsonEncode({
        "type": "auth_app",
        "firebaseToken": token,
        "robotId": robotId,
      }));

      // Listen messages
      _sub = _channel!.stream.listen((msg) {
        _handleMessage(msg);
      }, onDone: () {
        print("WebSocket disconnected.");
        _reconnect();
      }, onError: (e) {
        print("WebSocket error: $e");
        _reconnect();
      });

    } catch (e) {
      print("WS connection failed: $e");
      _reconnect();
    }
  }

  //Parse message
  static void _handleMessage(String msg) {
    try {
      final data = jsonDecode(msg);

      if (data["type"] == "telemetry") {
        onTelemetry?.call(Map<String, dynamic>.from(data["payload"]));
      }

      print("WS message: $data");

    } catch (e) {
      print("Invalid WS message: $msg");
    }
  }


  // Send Control Commands
  static void sendControl(Map<String, dynamic> cmd) {
    if (!_isConnected || _channel == null) {
      print("WebSocket not connected");
      return;
    }

    _channel!.sink.add(jsonEncode(cmd));
  }


  // Joystick command
  static void sendJoystick(double x, double y) {
    sendControl({
      "type": "control",
      "payload": {
        "x": x,
        "y": y,
      },
    });
  }

  static bool get isConnected => _isConnected;

  // Reconnect
  static void _reconnect() {
    _isConnected = false;
    Future.delayed(const Duration(seconds: 2), () {
      if (_robotId != null) connect(_robotId!);
    });
  }

  // Close conn

  static void close() {
    _isConnected = false;
    _sub?.cancel();
    _channel?.sink.close();
  }
}
