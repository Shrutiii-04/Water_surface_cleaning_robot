import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:firebase_auth/firebase_auth.dart';
import 'config.dart';

class ApiService {
  static Future<Map<String, dynamic>> pairRobot(
      String robotId, String pairCode) async {
    final user = FirebaseAuth.instance.currentUser;

    final response = await http.post(
      Uri.parse("${AppConfig.backendBaseUrl}/pair"),
      headers: {"Content-Type": "application/json"},
      body: jsonEncode({
        "userUid": user!.uid,
        "robotId": robotId,
        "pairCode": pairCode,
      }),
    );

    if (response.statusCode != 200) {
      throw Exception("Pairing failed");
    }

    return jsonDecode(response.body);
  }

  static Future<Map<String, dynamic>> getMqttToken(String robotId) async {
    final user = FirebaseAuth.instance.currentUser;

    final response = await http.post(
      Uri.parse("${AppConfig.backendBaseUrl}/mqtt-token"),
      headers: {"Content-Type": "application/json"},
      body: jsonEncode({
        "userUid": user!.uid,
        "robotId": robotId,
      }),
    );

    return jsonDecode(response.body);
  }
}
