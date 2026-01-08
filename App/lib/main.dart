import 'package:flutter/material.dart';
import 'package:water_surface_robot_app/screens/robot_startup.dart';
import 'theme/app_theme.dart';
import 'screens/splash_screen.dart';
import 'screens/login_screen.dart';
import 'screens/main_navigation.dart';
import 'screens/mode_selection.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_auth/firebase_auth.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp();

  runApp(const WSCRApp());
}

class WSCRApp extends StatelessWidget {
  const WSCRApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'WSCR',
      debugShowCheckedModeBanner: false,
      theme: AppTheme.lightTheme,
      darkTheme: AppTheme.darkTheme,
      themeMode: ThemeMode.dark,

      home: StreamBuilder<User?>(
        stream: FirebaseAuth.instance.authStateChanges(),
        builder: (context, snapshot) {

          if (snapshot.connectionState == ConnectionState.waiting) {
            return const SplashScreen();
          }

          // Logged in move to Mode Select
          if (snapshot.hasData) {
            return const RobotStartup();
          }

          // Logged out go back to Login
          return const LoginScreen();
        },
      ),
    );
  }
}
