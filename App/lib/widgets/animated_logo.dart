import 'package:flutter/material.dart';

class AnimatedLogo extends StatefulWidget {
  final double size;
  const AnimatedLogo({super.key, this.size = 120});

  @override
  State<AnimatedLogo> createState() => _AnimatedLogoState();
}

class _AnimatedLogoState extends State<AnimatedLogo> with SingleTickerProviderStateMixin {
  late final AnimationController _ctrl;

  @override
  void initState() {
    super.initState();
    _ctrl = AnimationController(vsync: this, duration: const Duration(seconds: 2))..repeat(reverse: true);
  }

  @override
  void dispose() {
    _ctrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _ctrl,
      builder: (context, child) {
        final glow = 0.6 + 0.4 * _ctrl.value;
        return Container(
          width: widget.size,
          height: widget.size,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            gradient: RadialGradient(
              colors: [Color.lerp(Colors.cyanAccent, Colors.blue.shade200, _ctrl.value)!, Colors.transparent],
              stops: const [0.0, 0.9],
            ),
            boxShadow: [
              BoxShadow(
                color: Colors.cyanAccent.withOpacity(glow),
                blurRadius: 24 * glow,
                spreadRadius: 6 * glow,
              )
            ],
          ),
          child: Center(
            child: Icon(
              Icons.water_drop,
              size: widget.size * 0.5,
              color: Colors.white,
            ),
          ),
        );
      },
    );
  }
}
