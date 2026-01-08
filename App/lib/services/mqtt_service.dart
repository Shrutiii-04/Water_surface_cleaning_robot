import 'dart:convert';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'api_service.dart';

class MqttService {
  static MqttServerClient? _client;
  static String? _currentRobotId;

  static Function(Map<String, dynamic>)? telemetryCallback;

  // Conn
  static Future<void> connect(String robotId) async {
    _currentRobotId = robotId;

    if (_client != null &&
        _client!.connectionStatus?.state == MqttConnectionState.connected) {
      return;
    }

    // Fetch token and broker info
    final tokenData = await ApiService.getMqttToken(robotId);

    final username = tokenData["mqttUsername"];
    final password = tokenData["token"];
    final brokerUrl = tokenData["mqttBroker"];


    final cleaned = brokerUrl.replaceAll("mqtt://", "");
    final parts = cleaned.split(":");

    final host = parts[0];
    final port = int.parse(parts[1]);

    print("Connecting to MQTT broker $host:$port");

    final client = MqttServerClient(host, username);
    client.port = port;

    client.keepAlivePeriod = 20;
    client.setProtocolV311();
    client.logging(on: false);

    client.onDisconnected = _onDisconnected;

    client.connectionMessage = MqttConnectMessage()
        .authenticateAs(username, password)
        .withClientIdentifier("app-${DateTime.now().millisecondsSinceEpoch}")
        .startClean();

    try {
      await client.connect();
      _client = client;

      print("MQTT connected successfully");

      _subscribeTelemetry(robotId);
    } catch (e) {
      print("MQTT connection failed: $e");
    }
  }


  // reconn
  static void _onDisconnected() {
    print("MQTT reconnecting");
    if (_currentRobotId != null) {
      Future.delayed(const Duration(seconds: 2), () {
        connect(_currentRobotId!);
      });
    }
  }

  // Subscribe topic
  static void _subscribeTelemetry(String robotId) {
    final topic = "robot/$robotId/telemetry";

    _client?.subscribe(topic, MqttQos.atLeastOnce);

    _client?.updates?.listen((events) {
      final recMsg = events[0];
      final MqttPublishMessage msg = recMsg.payload as MqttPublishMessage;
      final payload = MqttPublishPayload.bytesToStringAsString(msg.payload.message);


      try {
        final map = jsonDecode(payload);
        print("DECODED JSON → $map");
        telemetryCallback?.call(map);
      } catch (e) {
        print("Invalid telemetry JSON: $payload");
      }
    });

    print("Subscribed to telemetry → $topic");
  }

  // Pub mission
  static void publishMission(String robotId, Map<String, dynamic> geojson) {
    if (_client == null ||
        _client!.connectionStatus?.state != MqttConnectionState.connected) {
      print("Mission not sent.");
      return;
    }

    final topic = "robot/$robotId/cmd/mission";

    final builder = MqttClientPayloadBuilder();
    builder.addString(jsonEncode(geojson));

    _client!.publishMessage(topic, MqttQos.atLeastOnce, builder.payload!);

    print("Mission published → $topic");
  }
}
