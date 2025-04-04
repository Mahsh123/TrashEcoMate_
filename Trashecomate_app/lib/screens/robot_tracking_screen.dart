import 'package:flutter/material.dart';
import 'package:firebase_database/firebase_database.dart';

class RobotTrackingScreen extends StatefulWidget {
  const RobotTrackingScreen({Key? key}) : super(key: key);

  @override
  _RobotTrackingScreenState createState() => _RobotTrackingScreenState();
}

class _RobotTrackingScreenState extends State<RobotTrackingScreen> {
  final DatabaseReference _robotRef = FirebaseDatabase.instance.ref('robot');

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Robot Tracking'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Current Robot Locations',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 20),
            Expanded(
              child: StreamBuilder(
                stream: _robotRef.onValue,
                builder: (context, AsyncSnapshot<DatabaseEvent> snapshot) {
                  // Show loading indicator while waiting for data
                  if (snapshot.connectionState == ConnectionState.waiting) {
                    return const Center(child: CircularProgressIndicator());
                  }
                  // Handle errors
                  else if (snapshot.hasError) {
                    return Center(child: Text('Error: ${snapshot.error}'));
                  }
                  // Handle no data
                  else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                    return const Center(child: Text('No robot data available'));
                  }
                  // Data is available
                  else {
                    final data = snapshot.data!.snapshot.value as Map<dynamic, dynamic>;
                    final status = data['status'] ?? 'Unknown';
                    return ListView.builder(
                      itemCount: 1, // Single robot for this example
                      itemBuilder: (context, index) {
                        return _buildRobotCard(
                          context,
                          'Robot 1', // Static name, adjust if dynamic data available
                          'Location 1', // Static location, adjust if dynamic data available
                          'Status: $status', // Real-time status from Firebase
                        );
                      },
                    );
                  }
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildRobotCard(BuildContext context, String robotName, String location, String status) {
    return Card(
      elevation: 4,
      margin: const EdgeInsets.symmetric(vertical: 8.0),
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  robotName,
                  style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 4),
                Text(
                  location,
                  style: const TextStyle(color: Colors.grey),
                ),
              ],
            ),
            Column(
              crossAxisAlignment: CrossAxisAlignment.end,
              children: [
                Text(
                  status,
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: status.contains("Active") ? Colors.green : Colors.orange,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}