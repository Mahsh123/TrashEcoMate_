import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:intl/intl.dart';

class AnalyticsScreen extends StatefulWidget {
  const AnalyticsScreen({Key? key}) : super(key: key);

  @override
  _AnalyticsScreenState createState() => _AnalyticsScreenState();
}

class _AnalyticsScreenState extends State<AnalyticsScreen> {
  final wasteData = {
    "college": "TOMS College of Engineering",
    "duration": "March 24 – March 30, 2025",
    "unit": "Kilograms",
    "data": [
      {
        "date": "2025-03-24",
        "location": "Canteen Area",
        "organic": 22.4,
        "plastic": 8.5,
        "metal": 1.0,
        "eWaste": 0.2,
        "total": 32.1
      },
      {
        "date": "2025-03-24",
        "location": "Hostel Block A",
        "organic": 18.0,
        "plastic": 5.2,
        "metal": 0.8,
        "eWaste": 0.1,
        "total": 24.1
      },
      {
        "date": "2025-03-25",
        "location": "Canteen Area",
        "organic": 20.8,
        "plastic": 7.0,
        "metal": 1.2,
        "eWaste": 0.3,
        "total": 29.3
      },
      {
        "date": "2025-03-25",
        "location": "Hostel Block A",
        "organic": 15.3,
        "plastic": 4.8,
        "metal": 0.5,
        "eWaste": 0.2,
        "total": 20.8
      },
    ]
  };

  String _selectedLocation = 'All Locations';
  List<String> _locations = ['All Locations'];
  String _selectedTimeFrame = 'Daily';
  final List<String> _timeFrameOptions = ['Daily', 'By Location'];

  @override
  void initState() {
    super.initState();
    Set<String> locationSet = {};
    for (var item in wasteData['data'] as List) {
      locationSet.add(item['location'] as String);
    }
    _locations = ['All Locations', ...locationSet.toList()..sort()];
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Waste Analytics'),
        elevation: 2.0,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: SingleChildScrollView(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              _buildHeader(),
              const SizedBox(height: 20),
              _buildFilters(),
              const SizedBox(height: 20),
              _buildSummaryCards(),
              const SizedBox(height: 20),
              _buildChartSection(),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildHeader() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          wasteData['college'] as String,
          style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 4),
        Text(
          'Waste Management Analytics (${wasteData['duration']})',
          style: TextStyle(fontSize: 16, color: Colors.grey[600]),
        ),
      ],
    );
  }

  Widget _buildFilters() {
    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Filters',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  child: DropdownButtonFormField<String>(
                    decoration: const InputDecoration(
                      labelText: 'Location',
                      border: OutlineInputBorder(),
                    ),
                    value: _selectedLocation,
                    items: _locations.map((String location) {
                      return DropdownMenuItem<String>(
                        value: location,
                        child: Text(location),
                      );
                    }).toList(),
                    onChanged: (newValue) {
                      setState(() {
                        _selectedLocation = newValue!;
                      });
                    },
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: DropdownButtonFormField<String>(
                    decoration: const InputDecoration(
                      labelText: 'Time Frame',
                      border: OutlineInputBorder(),
                    ),
                    value: _selectedTimeFrame,
                    items: _timeFrameOptions.map((String option) {
                      return DropdownMenuItem<String>(
                        value: option,
                        child: Text(option),
                      );
                    }).toList(),
                    onChanged: (newValue) {
                      setState(() {
                        _selectedTimeFrame = newValue!;
                      });
                    },
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSummaryCards() {
    List filteredData = _getFilteredData();

    double totalWaste = 0;
    double totalOrganic = 0;
    double totalPlastic = 0;

    for (var item in filteredData) {
      totalWaste += (item['total'] as double);
      totalOrganic += (item['organic'] as double);
      totalPlastic += (item['plastic'] as double);
    }

    Map<String, double> locationWaste = {};
    for (var item in filteredData) {
      String location = item['location'] as String;
      double waste = item['total'] as double;
      locationWaste[location] = (locationWaste[location] ?? 0) + waste;
    }

    String hotspotLocation = '';
    double maxWaste = 0;
    locationWaste.forEach((location, waste) {
      if (waste > maxWaste) {
        maxWaste = waste;
        hotspotLocation = location;
      }
    });

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text(
          'Summary',
          style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 10),
        GridView.count(
          crossAxisCount: 2,
          crossAxisSpacing: 10,
          mainAxisSpacing: 10,
          shrinkWrap: true,
          physics: const NeverScrollableScrollPhysics(),
          children: [
            _buildSummaryCard(
              'Total Waste Collected',
              '${totalWaste.toStringAsFixed(1)} ${wasteData['unit']}',
              Icons.delete_outline,
              Colors.red,
            ),
            _buildSummaryCard(
              'Organic Waste',
              '${totalOrganic.toStringAsFixed(1)} ${wasteData['unit']}',
              Icons.eco_outlined,
              Colors.green,
            ),
            _buildSummaryCard(
              'Plastic Waste',
              '${totalPlastic.toStringAsFixed(1)} ${wasteData['unit']}',
              Icons.shopping_bag_outlined,
              Colors.blue,
            ),
            _buildSummaryCard(
              'Waste Hotspot',
              hotspotLocation,
              Icons.location_on_outlined,
              Colors.orange,
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildSummaryCard(
      String title, String value, IconData icon, Color color) {
    return Card(
      elevation: 3,
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, size: 40, color: color),
            const SizedBox(height: 8),
            Text(
              title,
              textAlign: TextAlign.center,
              style: const TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 4),
            Text(
              value,
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: color,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildChartSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text(
          'Waste Distribution',
          style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 10),
        SizedBox(
          height: 300,
          child: Card(
            elevation: 3,
            child: Padding(
              padding: const EdgeInsets.all(16.0),
              child: _selectedTimeFrame == 'Daily'
                  ? _buildDailyWasteChart()
                  : _buildLocationChart(),
            ),
          ),
        ),
        const SizedBox(height: 20),
        const Text(
          'Waste Composition',
          style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
        const SizedBox(height: 10),
        SizedBox(
          height: 300,
          child: Card(
            elevation: 3,
            child: Padding(
              padding: const EdgeInsets.all(16.0),
              child: _buildPieChart(),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildDailyWasteChart() {
    List filteredData = _getFilteredData();

    Map<String, double> dailyTotals = {};
    for (var item in filteredData) {
      String date = item['date'] as String;
      DateTime dateObj = DateTime.parse(date);
      String formattedDate = DateFormat('MMM dd').format(dateObj);

      dailyTotals[formattedDate] =
          (dailyTotals[formattedDate] ?? 0) + (item['total'] as double);
    }

    List<FlSpot> spots = [];
    List<String> bottomTitles = [];
    int index = 0;

    dailyTotals.forEach((date, total) {
      spots.add(FlSpot(index.toDouble(), total));
      bottomTitles.add(date);
      index++;
    });

    return LineChart(
      LineChartData(
        gridData: const FlGridData(show: true),
        titlesData: FlTitlesData(
          leftTitles: AxisTitles(
            sideTitles: SideTitles(
              showTitles: true,
              reservedSize: 40,
              getTitlesWidget: _getLeftTitles,
            ),
          ),
          bottomTitles: AxisTitles(
            sideTitles: SideTitles(
              showTitles: true,
              reservedSize: 30,
              getTitlesWidget: (value, meta) =>
                  _getBottomTitles(value, meta, bottomTitles),
            ),
          ),
          rightTitles: const AxisTitles(
            sideTitles: SideTitles(showTitles: false),
          ),
          topTitles: const AxisTitles(
            sideTitles: SideTitles(showTitles: false),
          ),
        ),
        borderData: FlBorderData(show: true),
        minX: 0,
        maxX: spots.isEmpty ? 0 : (spots.length - 1).toDouble(),
        lineBarsData: [
          LineChartBarData(
            spots: spots,
            isCurved: true,
            color: Colors.blue,
            barWidth: 3,
            isStrokeCapRound: true,
            dotData: const FlDotData(show: true),
            belowBarData: BarAreaData(
              show: true,
              color: Colors.blue.withOpacity(0.3),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildLocationChart() {
    List filteredData = _getFilteredData();

    Map<String, double> locationTotals = {};
    for (var item in filteredData) {
      String location = item['location'] as String;
      locationTotals[location] =
          (locationTotals[location] ?? 0) + (item['total'] as double);
    }

    List<BarChartGroupData> barGroups = [];
    List<String> locations = [];
    int index = 0;

    locationTotals.forEach((location, total) {
      barGroups.add(
        BarChartGroupData(
          x: index,
          barRods: [
            BarChartRodData(
              toY: total,
              color: Colors.blue,
              width: 20,
              borderRadius: const BorderRadius.only(
                topLeft: Radius.circular(6),
                topRight: Radius.circular(6),
              ),
            ),
          ],
        ),
      );
      locations.add(location);
      index++;
    });

    return BarChart(
      BarChartData(
        alignment: BarChartAlignment.spaceAround,
        maxY: barGroups.isEmpty
            ? 10
            : barGroups
                    .map((group) => group.barRods.first.toY)
                    .reduce((a, b) => a > b ? a : b) *
                1.2,
        gridData: const FlGridData(show: false),
        titlesData: FlTitlesData(
          leftTitles: AxisTitles(
            sideTitles: SideTitles(
              showTitles: true,
              reservedSize: 40,
              getTitlesWidget: _getLeftTitles,
            ),
          ),
          bottomTitles: AxisTitles(
            sideTitles: SideTitles(
              showTitles: true,
              reservedSize: 60,
              getTitlesWidget: (value, meta) =>
                  _getBottomTitles(value, meta, locations),
            ),
          ),
          rightTitles: const AxisTitles(
            sideTitles: SideTitles(showTitles: false),
          ),
          topTitles: const AxisTitles(
            sideTitles: SideTitles(showTitles: false),
          ),
        ),
        borderData: FlBorderData(show: false),
        barGroups: barGroups,
      ),
    );
  }

  Widget _buildPieChart() {
    List filteredData = _getFilteredData();

    double totalOrganic = 0;
    double totalPlastic = 0;
    double totalMetal = 0;
    double totalEWaste = 0;

    for (var item in filteredData) {
      totalOrganic += (item['organic'] as double);
      totalPlastic += (item['plastic'] as double);
      totalMetal += (item['metal'] as double);
      totalEWaste += (item['eWaste'] as double);
    }

    double total = totalOrganic + totalPlastic + totalMetal + totalEWaste;

    List<PieChartSectionData> sections = [];

    if (total > 0) {
      sections = [
        PieChartSectionData(
          color: Colors.green,
          value: totalOrganic,
          title: '${(totalOrganic / total * 100).toStringAsFixed(1)}%',
          radius: 80,
          titleStyle: const TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
        PieChartSectionData(
          color: Colors.blue,
          value: totalPlastic,
          title: '${(totalPlastic / total * 100).toStringAsFixed(1)}%',
          radius: 80,
          titleStyle: const TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
        PieChartSectionData(
          color: Colors.grey,
          value: totalMetal,
          title: '${(totalMetal / total * 100).toStringAsFixed(1)}%',
          radius: 80,
          titleStyle: const TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
        PieChartSectionData(
          color: Colors.red,
          value: totalEWaste,
          title: '${(totalEWaste / total * 100).toStringAsFixed(1)}%',
          radius: 80,
          titleStyle: const TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
      ];
    }

    return Column(
      children: [
        Expanded(
          child: PieChart(
            PieChartData(
              sections: sections,
              centerSpaceRadius: 40,
              sectionsSpace: 2,
            ),
          ),
        ),
        const SizedBox(height: 16),
        Wrap(
          spacing: 16,
          runSpacing: 8,
          children: [
            _buildLegendItem('Organic', Colors.green),
            _buildLegendItem('Plastic', Colors.blue),
            _buildLegendItem('Metal', Colors.grey),
            _buildLegendItem('E-Waste', Colors.red),
          ],
        ),
      ],
    );
  }

  Widget _buildLegendItem(String label, Color color) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 12,
          height: 12,
          color: color,
        ),
        const SizedBox(width: 4),
        Text(label),
      ],
    );
  }

  List _getFilteredData() {
    List data = wasteData['data'] as List;

    if (_selectedLocation != 'All Locations') {
      data = data.where((item) => item['location'] == _selectedLocation).toList();
    }

    return data;
  }

  Widget _getLeftTitles(double value, TitleMeta meta) {
    return Text(
      value.toInt().toString(),
      style: const TextStyle(fontSize: 10),
    );
  }

  Widget _getBottomTitles(double value, TitleMeta meta, List<String> titles) {
    if (value >= 0 && value < titles.length) {
      return Padding(
        padding: const EdgeInsets.only(top: 8.0),
        child: Text(
          titles[value.toInt()],
          style: const TextStyle(fontSize: 10),
          textAlign: TextAlign.center,
          maxLines: 2,
        ),
      );
    }
    return const Text('');
  }
}