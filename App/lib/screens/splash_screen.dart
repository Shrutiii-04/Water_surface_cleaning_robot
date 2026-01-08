import 'dart:async';
import 'package:flutter/material.dart';
import '../widgets/animated_logo.dart';
import 'login_screen.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({super.key});
  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen> {
  @override
  void initState() {
    super.initState();
    Timer(const Duration(seconds: 3), () {
      Navigator.pushReplacement(context, MaterialPageRoute(builder: (_) => const LoginScreen()));
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF071127),
      body: SafeArea(
        child: Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: const [
              AnimatedLogo(size: 140),
              SizedBox(height: 18),
              Text('WSCR', style: TextStyle(color: Colors.white, fontSize: 24, fontWeight: FontWeight.bold)),
              SizedBox(height: 8),
              Text('Water Surface Cleaning Robot', style: TextStyle(color: Colors.white70)),
              SizedBox(height: 24),
              CircularProgressIndicator(color: Color(0xFF00E5FF)),
            ],
          ),
        ),
      ),
    );
  }
}
